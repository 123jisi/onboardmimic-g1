"""Opt-in offline parity against trusted host source; no SDK, sockets or robot.

Extract only numerical functions from the manager AST so its service startup,
visualizers, device SDK and streamers cannot run. Pinocchio/Torch are host-only.
"""
import argparse
import ast
import contextlib
import hashlib
import io
import json
from pathlib import Path
import sys
import textwrap
from enum import Enum,IntEnum
from types import SimpleNamespace,MethodType
from typing import Dict
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial.transform import Rotation as R
from onboardmimic.backends.sonic.pico.kinematics import MODEL,key_frames,raw_three_points,ThreePointCalibration
from onboardmimic.backends.sonic.pico.pose_mapping import map_body,PARENTS,mix
from onboardmimic.backends.sonic.pico.session import PicoSession

def extract(path,names,env):
    tree=ast.parse(path.read_text())
    nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names]
    if {n.name for n in nodes}!=set(names):raise ValueError('Reference numerical functions changed')
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),env)

def compare(root,fixture=None):
    import torch
    import pinocchio as pin
    sys.path.insert(0,str(root))
    from gear_sonic.trl.utils import torch_transform as tt
    from gear_sonic.isaac_utils.rotations import remove_smpl_base_rot,smpl_root_ytoz_up
    from gear_sonic.trl.utils.rotation_conversion import decompose_rotation_aa
    tt.human_joints_info=torch.load(root/'gear_sonic/data/human/human_joints_info.pkl',map_location='cpu',weights_only=False)
    manager=root/'gear_sonic/scripts/pico_manager_thread_server.py'
    env=dict(np=np,torch=torch,sRot=R,R=R,Dict=Dict,smpl_root_ytoz_up=smpl_root_ytoz_up,
             remove_smpl_base_rot=remove_smpl_base_rot,decompose_rotation_aa=decompose_rotation_aa,
             OFFSETS=[R.from_euler('xyz',a,degrees=True) for a in ((0,0,-90),(90,0,0),(-90,0,180),(0,0,-90))])
    for name in ('angle_axis_to_quaternion','compute_human_joints','quat_apply','quat_inv','quaternion_to_angle_axis','quaternion_to_rotation_matrix'):env[name]=getattr(tt,name)
    extract(manager,('compute_from_body_poses','process_smpl_joints','_compute_rel_transform','_process_3pt_pose','_interp_pose_axis_angle','_quat_lerp_normalized'),env)
    fk_source=root/'gear_sonic/utils/teleop/vis/vr3pt_pose_visualizer.py'
    env.update(G1_FRAME_MAPPING=dict(left_wrist='left_wrist_yaw_link',right_wrist='right_wrist_yaw_link',torso='torso_link'),
               G1_KEY_FRAME_OFFSETS=dict(left_wrist=np.array([.18,-.025,0]),right_wrist=np.array([.18,.025,0]),torso=np.array([0,0,.35])))
    extract(fk_source,('get_g1_key_frame_poses',),env)
    tree=ast.parse(manager.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ThreePointPose')
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('_capture_calibration','_apply_calibration')]
    exec(compile(ast.Module(body=methods,type_ignores=[]),str(manager),'exec'),env)
    class PinModel:
        def __init__(self):
            self.model=pin.buildModelFromUrdf(str(root/MODEL['source']));self.data=self.model.createData()
            self.default_body_pose=pin.neutral(self.model)
        def get_configuration_from_actuated_joints(self,body_actuated_joint_values):
            q=pin.neutral(self.model)
            for name,value in zip(MODEL['body_joint_names'],body_actuated_joint_values):q[self.model.joints[self.model.getJointId(name)].idx_q]=value
            return q
        def cache_forward_kinematics(self,q,auto_clip=False):
            pin.forwardKinematics(self.model,self.data,q);pin.updateFramePlacements(self.model,self.data)
        def frame_placement(self,name):return self.data.oMf[self.model.getFrameId(name)]
    model=PinModel()
    # Original wrist code, verbatim, extracted from the PoseStreamer method.
    source=manager.read_text();start=source.index('        joint_pos = np.zeros(29)');end=source.index('        # Process SMPL pose',start)
    wrist_code=compile(textwrap.dedent(source[start:end]),str(manager),'exec')
    rng=np.random.default_rng(20261113);errors={k:0. for k in ('pose_rad','joints_m','root_rad','wrist_rad','fk_m','fk_rad','raw_vr_m','raw_vr_rad','calibration_m','calibration_rad','interpolation_rad')};rows=[]
    for _ in range(32):
        b=np.c_[rng.normal(0,.3,(24,3)),R.from_rotvec(rng.normal(0,.15,(24,3))).as_quat()]
        q=rng.uniform(-.2,.2,29)
        old=env['compute_from_body_poses'](PARENTS,torch.device('cpu'),b);new=map_body(b)
        expected=dict(smpl_pose=old['smpl_pose'].numpy().reshape(23,3)[:21],smpl_joints=old['smpl_joints_local'].numpy()[0],body_quat_w=old['global_orient_quat'].numpy()[0])
        env['use_pose']=expected['smpl_pose'];exec(wrist_code,env);expected['joint_pos']=env['joint_pos'].copy()
        def rotations(a,b,scalar_first=False):
            a=np.asarray(a);b=np.asarray(b)
            if scalar_first:a=a[..., [1,2,3,0]];b=b[..., [1,2,3,0]]
            return float(np.max((R.from_quat(a).inv()*R.from_quat(b)).magnitude()))
        errors['pose_rad']=max(errors['pose_rad'],float(np.max((R.from_rotvec(new['smpl_pose']).inv()*R.from_rotvec(expected['smpl_pose'])).magnitude())))
        errors['joints_m']=max(errors['joints_m'],float(np.max(abs(new['smpl_joints']-expected['smpl_joints']))))
        errors['root_rad']=max(errors['root_rad'],rotations(new['body_quat_w'],expected['body_quat_w'],True))
        errors['wrist_rad']=max(errors['wrist_rad'],float(np.max(abs(new['joint_pos']-expected['joint_pos']))))
        ref_fk=env['get_g1_key_frame_poses'](model,q=model.get_configuration_from_actuated_joints(q))
        fk=np.array([np.r_[ref_fk[k]['position'],ref_fk[k]['orientation_wxyz']] for k in ('left_wrist','right_wrist','torso')]);actual=key_frames(q)
        raw=env['_process_3pt_pose'](b);newraw=raw_three_points(b)
        host=SimpleNamespace(_calibration_neck_quat_inv=None,_robot_model=model,_override_robot_q=q,
                             TORSO_LINK_OFFSET_Z=.05,NECK_LINK_LENGTH=.35,log_prefix='offline')
        with contextlib.redirect_stdout(io.StringIO()):env['_capture_calibration'](host,raw)
        c=ThreePointCalibration();c.capture(b,q)
        moved=b.copy();moved[:,:3]+=rng.normal(0,.02,(24,3));moved[:,3:]=(R.from_rotvec(rng.normal(0,.05,(24,3)))*R.from_quat(b[:,3:])).as_quat()
        vr=env['_apply_calibration'](host,env['_process_3pt_pose'](moved));actualvr=c.apply(moved)
        for name,a,z in (('fk',actual,fk),('raw_vr',newraw,raw),('calibration',actualvr,vr)):
            errors[name+'_m']=max(errors[name+'_m'],float(np.max(abs(a[:,:3]-z[:,:3]))))
            errors[name+'_rad']=max(errors[name+'_rad'],rotations(a[:,3:],z[:,3:],True))
        mapped=map_body(moved);interp=mix(new,mapped,.37)
        hostinterp=env['_interp_pose_axis_angle'](new['smpl_pose'],mapped['smpl_pose'],.37)
        errors['interpolation_rad']=max(errors['interpolation_rad'],float(np.max((R.from_rotvec(interp['smpl_pose']).inv()*R.from_rotvec(hostinterp)).magnitude())))
        if len(rows)<8:rows.append(dict(body=b.tolist(),moved=moved.tolist(),q=q.tolist(),expected={k:v.tolist() for k,v in expected.items()},fk=fk.tolist(),vr=vr.tolist()))
    hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (manager,fk_source,root/MODEL['source'])}
    # Run the actual host planner method against numerical sinks, not the SDK.
    env.update(Enum=Enum,IntEnum=IntEnum,JOYSTICK_DEADZONE=.15,time=SimpleNamespace(time=lambda:0.,sleep=lambda _:None))
    classes=[n for n in tree.body if isinstance(n,ast.ClassDef) and n.name in ('LocomotionMode','StreamMode','YawAccumulator')]
    exec(compile(ast.Module(body=classes,type_ignores=[]),str(manager),'exec'),env)
    planner=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PlannerStreamer')
    method=next(n for n in planner.body if isinstance(n,ast.FunctionDef) and n.name=='run_once')
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(manager),'exec'),env)
    control=lambda:dict(axes=[0.,0.],menu=False,click=False,primary=False,secondary=False,trigger=0.,grip=0.)
    snap=dict(ready=True,controllers_ready=True,raw_body=rows[0]['body'],controllers={'left':control(),'right':control()})
    target=PicoSession(SimpleNamespace(snapshot=lambda:snap))
    target.enter('planner',np.zeros(29),True);target.released=True
    first=map_body(rows[0]['body']);snap['fields']={k:np.stack([v]*5) for k,v in first.items()}
    captured={}
    def sink(mode,movement,facing,**kw):
        captured.update(mode=mode,movement=movement,facing=facing,**kw);return b'no-network'
    env.update(xrt=SimpleNamespace(get_time_stamp_ns=lambda:1),get_abxy_buttons=lambda:(False,)*4,build_planner_message=sink)
    errors.update(planner_movement=0.,planner_facing=0.,planner_speed=0.,pose_heading_rad=0.)
    cases=0
    for lx,ly in ((0.,.7),(.4,.7),(-.8,-.2),(.8,.8)):
        for rx in (0.,.05,-.3,.8):
            for gait in (0,1,2,3,19):
                env['get_controller_axes']=lambda:(lx,ly,rx,0.)
                with contextlib.redirect_stdout(io.StringIO()):yaw=env['YawAccumulator']()
                host=SimpleNamespace(last_xrt_timestamp=None,prev_ab=False,prev_xy=False,mode=env['LocomotionMode'](gait),yaw_accumulator=yaw,dt=.05,last_send=0.,socket=SimpleNamespace(send=lambda _:None))
                env['run_once'](host,env['StreamMode'].PLANNER)
                snap['controllers']['left']['axes']=[lx,ly];snap['controllers']['right']['axes']=[rx,0.]
                target.mode='planner';target.gait=gait;target.heading=0.;target.last=0.;actual=target.reference(.05)
                for name,a,z in (('planner_movement',actual['movement_direction'],captured['movement']),('planner_facing',actual['facing_direction'],captured['facing']),('planner_speed',actual['speed'],captured['speed'])):
                    errors[name]=max(errors[name],float(np.max(abs(np.asarray(a)-z))))
                # POSE transmits dyaw even inside the host's facing deadzone.
                target.mode='pose';target.last=0.;wire=target.reference(.05)
                header=json.loads(wire[4:1284].rstrip(b'\0 '));offset=1284
                for f in header['fields']:
                    dtype=np.dtype({'f32':'<f4','f64':'<f8','i64':'<i8'}[f['dtype']]);count=int(np.prod(f['shape']))
                    if f['name']=='heading_increment':
                        delta=float(np.frombuffer(wire,dtype=dtype,count=1,offset=offset)[0]);break
                    offset+=count*dtype.itemsize
                errors['pose_heading_rad']=max(errors['pose_heading_rad'],abs(yaw.yaw_angle_change()-delta))
                cases+=1
    report=dict(frames=32,controller_cases=cases,seed=20261113,max_errors=errors,tolerance=5e-5,source_sha256=hashes,hardware=False)
    if max(errors.values())>5e-5:raise AssertionError(report)
    if fixture:
        fixture.parent.mkdir(parents=True,exist_ok=True);fixture.write_text(json.dumps(dict(source_sha256=hashes,rows=rows),separators=(',',':'))+'\n')
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference-root',type=Path,required=True);p.add_argument('--fixture',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    report=compare(a.reference_root.resolve(),a.fixture);text=json.dumps(report,indent=2)
    if a.output:a.output.write_text(text+'\n')
    print(text)
