"""Compare PN mapping with trusted host math only; never import live SDK/ROS/main."""
import argparse
import __future__
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation as R
from onboardmimic.backends.sonic.pnlink.mapping import map_joints, BODY_NAMES


def compare(reference, fixture=None):
    import torch
    sys.path.insert(0,str(reference))
    source=reference/'gear_sonic/scripts/mocap_live_play_smplx.py'
    tree=ast.parse(source.read_text())
    selected=[]
    for node in tree.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='JOINT_MAP' for t in node.targets):selected.append(node)
        elif isinstance(node,ast.FunctionDef) and node.name in ('_quat_wxyz_from_obj','_pos_xyz_from_obj'):selected.append(node)
        elif isinstance(node,ast.ClassDef):
            selected += [n for n in node.body if isinstance(n,ast.FunctionDef) and n.name=='_avatar_to_params']
    ns=dict(np=np,R=R,torch=torch,MCPAvatar=object)
    exec(compile(ast.fix_missing_locations(ast.Module(body=selected,type_ignores=[])),str(source),'exec', flags=__future__.annotations.compiler_flag),ns)
    publisher_path=reference/'gear_sonic/scripts/publish_smpl_kimodo.py'
    spec=importlib.util.spec_from_file_location('pnlink_host_math',publisher_path)
    pub=importlib.util.module_from_spec(spec);spec.loader.exec_module(pub)
    from gear_sonic.trl.utils import torch_transform
    torch_transform.human_joints_info=torch.load(reference/'gear_sonic/data/human/human_joints_info.pkl',weights_only=False)
    rng=np.random.default_rng(607)
    errors={k:0. for k in ('smpl_pose','smpl_joints','body_quat_w','joint_pos','joint_vel')};samples=[]
    for i in range(32):
        aa=rng.normal(size=(22,3))*(0.02 if i<4 else .35)
        rows=[dict(name=name,rotation_wxyz=R.from_rotvec(v).as_quat()[[3,0,1,2]].tolist(),position_xyz=[0.,100.,0.]) for name,v in zip(BODY_NAMES,aa)]
        if i%2:rows=[row for row in rows if row['name'] not in ('LeftTiptoe','RightTiptoe')]
        class Joint:
            def __init__(self,row):self.row=row
            def get_name(self):return self.row['name']
            def get_local_rotation(self):return self.row['rotation_wxyz']
            def get_local_position(self):return self.row['position_xyz']
        avatar=SimpleNamespace(get_joints=lambda:[Joint(r) for r in reversed(rows)])
        params=ns['_avatar_to_params'](SimpleNamespace(scale=.009),avatar)
        pose=pub._normalize_smpl_pose(params)
        expected=dict(smpl_pose=pose.numpy()[0],
                      smpl_joints=pub._compute_smpl_joints_local(params,pose).numpy()[0],
                      body_quat_w=pub._compute_body_quat_sequence(pub._extract_global_orient(params,1))[0],
                      joint_pos=pub._build_wrist_joint_pos_from_smpl(pose.numpy()[0]),
                      joint_vel=np.zeros(29,dtype=np.float32))
        actual=map_joints(rows)
        if np.dot(actual['body_quat_w'],expected['body_quat_w'])<0:expected['body_quat_w']*=-1
        for key in errors:errors[key]=max(errors[key],float(np.max(np.abs(actual[key]-expected[key]))))
        if i<4:samples.append(dict(joints=rows,expected={k:v.tolist() for k,v in expected.items()}))
    result=dict(frames=32,max_abs=errors,tolerance=1e-5,passed=max(errors.values())<=1e-5,
                sources={str(p.relative_to(reference)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (source,publisher_path)},
                sdk_loaded=False,robot_output_enabled=False)
    if not result['passed']:raise AssertionError(result)
    if fixture:fixture.write_text(json.dumps(dict(validation=result,samples=samples),indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference-root',type=Path,required=True)
    p.add_argument('--fixture',type=Path)
    p.add_argument('--output',type=Path)
    a=p.parse_args();result=compare(a.reference_root.resolve(),a.fixture)
    if a.output:a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
