"""Offline production merger/reference/TRT replay. Never constructs robot control.

GPU observations use a fixed nominal upright robot history, not recorded LowState.
Only production g1/mode0 and SMPL/mode2 references are supported. No mode fallback.
"""
import argparse,hashlib,json,re,subprocess,sys,tempfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from onboardmimic.backends.sonic.csv_player import CSVClip,CSVPlayer
from onboardmimic.backends.sonic.smpl_player import SMPLClip
from onboardmimic.backends.sonic.wire import HEADER_SIZE

METHODS=('UpdateHeadingState','ComputeApplyDeltaHeading','GatherMotionAnchorOrientationMutiFrame',
         'GatherMotionJointPositionsMultiFrame','GatherMotionJointVelocitiesMultiFrame','GatherMotionSmplJointsMultiFrame')
DIMS=dict(encoder_mode_4=4,motion_joint_positions_10frame_step5=290,motion_joint_velocities_10frame_step5=290,
          motion_root_z_position_10frame_step5=10,motion_root_z_position=1,motion_anchor_orientation=6,
          motion_anchor_orientation_10frame_step5=60,motion_joint_positions_lowerbody_10frame_step5=120,
          motion_joint_velocities_lowerbody_10frame_step5=120,vr_3point_local_target=9,vr_3point_local_orn_target=12,
          smpl_joints_10frame_step1=720,smpl_anchor_orientation_10frame_step1=60,motion_joint_positions_wrists_10frame_step1=60)

def unpack(raw):
    h=json.loads(raw[4:4+HEADER_SIZE].rstrip(b'\0'));data=memoryview(raw)[4+HEADER_SIZE:];out={'version':h['v']}
    for f in h['fields']:
        dt={'f32':'<f4','f64':'<f8','i64':'<i8','i32':'<i4'}[f['dtype']]
        a=np.frombuffer(data,dtype=dt,count=int(np.prod(f['shape']))).reshape(f['shape'])
        data=data[a.nbytes:]
        if f['name']=='body_quat_w':out['body_quat']=a[:,None,:].tolist()
        elif f['name'] in ('joint_pos','joint_vel','body_quat','frame_index','smpl_pose','smpl_joints'):out[f['name']]=a.tolist()
    return out

def layout(deploy):
    text=(deploy/'policy/release/observation_config.yaml').read_text()
    section=text.split('encoder_observations:',1)[1].split('encoder_modes:',1)[0]
    fields=[];offset=0
    for name in re.findall(r'name:\s*"([^"]+)"',section):
        if name not in DIMS:raise ValueError('unsupported production observation '+name)
        fields.append(dict(name=name,offset=offset,dimension=DIMS[name]));offset+=DIMS[name]
    return dict(dimension=offset,fields=fields)

def compile_replay(source,area):
    deploy=source/'onboardmimic/backends/sonic/deploy'
    include=deploy/'src/g1/g1_deploy_onnx_ref/include'
    cpp=(deploy/'src/g1/g1_deploy_onnx_ref/src/g1_deploy_onnx_ref.cpp').read_text()
    generated=[];hashes={}
    for name in METHODS:
        m=re.search(r'^    (?:void|bool|std::array<double, 4>) '+name+r'\(',cpp,re.M)
        if not m:raise ValueError('production gather unavailable: '+name)
        end=cpp.index('\n    }',m.start())+len('\n    }')
        body=cpp[m.start():end];generated.append(body);hashes[name]=hashlib.sha256(body.encode()).hexdigest()
    (area/'replay_gather_generated.hpp').write_text('\n'.join(generated))
    binary=area/'replay'
    cmd=['g++','-std=c++17','-O2','-pthread','-I'+str(include),'-I'+str(source/'var/deps/include'),
         '-I'+str(ROOT/'var/deps/include'),'-I'+str(deploy/'onboard'),'-I'+str(area),
         '-I/usr/local/cuda/include',str(ROOT/'scripts/native_file_replay.cpp'),'-L/usr/local/cuda/lib64',
         '-Wl,-rpath,/usr/local/cuda/lib64','-lnvinfer','-lcudart','-o',str(binary)]
    subprocess.run(cmd,check=True)
    needed=subprocess.check_output(['readelf','-d',str(binary)],text=True)
    if any(x in needed for x in ('libddsc','libunitree','libzmq')):raise RuntimeError('unexpected control/network dependency')
    return binary,deploy,hashes

def ticks(clip,horizon,drops=(),pause=False,full=False):
    p=CSVPlayer();p.lookahead_frames=horizon;p.load(clip);p.start(10.);p.confirm(10.)
    result=[]
    for tick in range(clip.frames+(10 if pause else 0)):
        if pause and tick==40:p.pause()
        if pause and tick==50:p.resume(10.+tick*.02-.02)
        raw=p.packet(10.+tick*.02)
        if raw is None:break
        packet=unpack(raw)
        result.append(dict(tick=tick,packet=None if tick in drops else packet))
    if full:
        # Build a complete reference sequence with a held tail; advance its cursor normally.
        full_packet=unpack(CSVPlayerPacket(clip))
        result=[dict(tick=i,packet=full_packet if i==0 else None) for i in range(clip.frames)]
    return result

def CSVPlayerPacket(clip):
    p=CSVPlayer();p.lookahead_frames=clip.frames+46;p.load(clip);p.start(10.);p.confirm(10.)
    return p.packet(10.)

def summarize(out):
    ts=out['traces'];frames=[x['consumed_frame'] for x in ts]
    return dict(ticks=len(ts),unexpected_resets=out['unexpected_catchup_resets'],
                max_cursor_jump=max([b-a for a,b in zip(frames,frames[1:])] or [0]),
                final_consumed_frame=frames[-1] if frames else None)

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--source-root',type=Path,default=ROOT)
    ap.add_argument('--gpu',action='store_true');ap.add_argument('--output',type=Path,default=ROOT/'var/unification/file-stream-replay.json')
    a=ap.parse_args();a.output.parent.mkdir(parents=True,exist_ok=True)
    clips=[CSVClip(ROOT/'var/motions'/name,fps=50,joint_order='sonic_reference') for name in ('wave_g1_fixed_20261113','pingpang_g1_fixed_20261113')]
    clips.append(SMPLClip(ROOT/'var/motions/genmo_pt_wave.npz'))
    report=dict(source_root=str(a.source_root),gpu=a.gpu,motor_output=False,hand_output=False,synthetic_fixed_robot_history=True,
                limitation='No accident LowState history or physics. Hardware targets precede final safety filters. Wire fields decoded in Python; production network endpoint not constructed.',motions=[])
    traces=a.output.parent/(a.output.stem+'-traces');traces.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        area=Path(tmp);binary,deploy,hashes=compile_replay(a.source_root,area);report['production_gather_sha256']=hashes
        report['merger_sha256']=hashlib.sha256((deploy/'src/g1/g1_deploy_onnx_ref/include/input_interface/streamed_motion_merger.hpp').read_bytes()).hexdigest()
        for clip in clips:
            record=dict(name=clip.path.name,identity=clip.metadata(),scenarios={});outputs={}
            cases=[('legacy_single_drop107',1,(107,),False,False),('window64_drop107',64,(107,),False,False),
                   ('window64_drop10',64,tuple(range(100,110)),False,False),('window64_pause',64,(),True,False),
                   ('full_reference',64,(),False,True),('window64_clean',64,(),False,False)]
            for name,horizon,drops,pause,full in cases:
                data=dict(gpu=a.gpu,engines=str(deploy/'onboard/engines'),encoder_layout=layout(deploy),ticks=ticks(clip,horizon,drops,pause,full))
                inp=area/'input.json';inp.write_text(json.dumps(data));target=traces/(clip.path.name+'-'+name+'.json')
                subprocess.run([str(binary),str(inp),str(target)],check=True,stdout=subprocess.DEVNULL)
                out=json.loads(target.read_text());outputs[name]=out;record['scenarios'][name]=summarize(out)
            for name in ('window64_drop107','window64_drop10','window64_pause','window64_clean'):
                assert record['scenarios'][name]['unexpected_resets']==0,(clip.path.name,name)
                assert record['scenarios'][name]['max_cursor_jump']<=1,(clip.path.name,name)
                if name != 'window64_pause':assert record['scenarios'][name]['final_consumed_frame']==clip.frames-1,(clip.path.name,name,'lag')
            clean=outputs['window64_clean']['traces'];reference=outputs['full_reference']['traces']
            err=max(float(np.max(np.abs(np.asarray(x['encoder_observation'])-np.asarray(y['encoder_observation'])))) for x,y in zip(clean,reference))
            record['window_vs_full_encoder_max_abs_error']=err
            assert err<1e-6,(clip.path.name,err)
            if a.gpu:
                for field in ('token','raw_action','unrestricted_hardware_q_target'):
                    record['window_vs_full_'+field+'_max_abs_error']=max(float(np.max(np.abs(np.asarray(x[field])-np.asarray(y[field])))) for x,y in zip(clean,reference))
                    assert record['window_vs_full_'+field+'_max_abs_error']<1e-5
            report['motions'].append(record)
    report['passed']=True;a.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('motions','production_gather_sha256')},indent=2))
    for m in report['motions']:print(m['name'],json.dumps(m['scenarios']))

if __name__=='__main__':main()
