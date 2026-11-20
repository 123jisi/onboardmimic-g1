"""Offline comparison with trusted legacy Kimodo publisher; never calls main."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from onboardmimic.backends.sonic.kimodo_import import import_kimodo, load_amass
from onboardmimic.backends.sonic.motion_import import smpl_arrays
from onboardmimic.backends.sonic.smpl_player import SMPLClip


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sonic',type=Path,required=True)
    p.add_argument('--input',type=Path,required=True)
    p.add_argument('--output-json',type=Path)
    a=p.parse_args()
    import torch
    sys.path.insert(0,str(a.sonic))
    source=a.sonic/'gear_sonic/scripts/publish_smpl_kimodo.py'
    spec=importlib.util.spec_from_file_location('legacy_kimodo',source)
    publisher=importlib.util.module_from_spec(spec);spec.loader.exec_module(publisher)
    from gear_sonic.trl.utils import torch_transform
    torch_transform.human_joints_info=torch.load(a.sonic/'gear_sonic/data/human/human_joints_info.pkl',weights_only=False)
    params=publisher._load_flat_smpl_dict(a.input,'auto')
    pose=publisher._normalize_smpl_pose(params)
    old=dict(smpl_pose=pose.numpy(),smpl_joints=publisher._compute_smpl_joints_local(params,pose).numpy(),
             body_quat_w=publisher._compute_body_quat_sequence(publisher._extract_global_orient(params,len(pose))),
             joint_pos=np.stack([publisher._build_wrist_joint_pos_from_smpl(x.numpy()) for x in pose]),
             joint_vel=np.zeros((len(pose),29),dtype=np.float32))
    numeric,fps,_=load_amass(a.input)
    new=smpl_arrays(numeric,wrist_mapping='kimodo')
    errors={k:float(np.max(np.abs(new[k]-old[k]))) for k in old}
    if max(errors.values())>1e-5:raise AssertionError(errors)
    with tempfile.TemporaryDirectory() as tmp:
        path=Path(tmp)/'cache.npz';meta=import_kimodo(a.input,path);clip=SMPLClip(path)
        with np.load(path,allow_pickle=False) as z:
            for k in old:np.testing.assert_allclose(z['source_'+k],old[k],atol=1e-5,rtol=0)
        # Independently evaluate legacy samples at the output timestamps.
        from scipy.spatial.transform import Rotation as R, Slerp
        ti=np.arange(len(pose))/fps;to=np.linspace(0,ti[-1],clip.frames)
        expected_pose=np.stack([Slerp(ti,R.from_rotvec(old['smpl_pose'][:,j]))(to).as_rotvec() for j in range(21)],axis=1)
        expected_joints=np.stack([np.interp(to,ti,c) for c in old['smpl_joints'].reshape(len(pose),-1).T],axis=-1).reshape(clip.frames,24,3)
        expected_quat=Slerp(ti,R.from_quat(old['body_quat_w'][:,[1,2,3,0]]))(to)
        actual_quat=R.from_quat(clip.quat[:,[1,2,3,0]])
        expected_wrist=np.stack([publisher._build_wrist_joint_pos_from_smpl(x.astype(np.float32)) for x in expected_pose])
        resampled_errors=dict(smpl_pose=float(abs(expected_pose-clip.arrays['smpl_pose']).max()),
                              smpl_joints=float(abs(expected_joints-clip.arrays['smpl_joints']).max()),
                              root_rotation_radians=float((expected_quat*actual_quat.inv()).magnitude().max()),
                              joint_pos=float(abs(expected_wrist-clip.q).max()))
        if max(resampled_errors.values())>1e-5:raise AssertionError(resampled_errors)
    result=dict(input=str(a.input),input_sha256=hashlib.sha256(a.input.read_bytes()).hexdigest(),
                publisher_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),source_frames=len(pose),
                source_fps=fps,output_frames=clip.frames,output_fps=50,playback_duration=clip.duration,
                original_frame_max_abs=errors,resampled_max_abs=resampled_errors,tolerance=1e-5,passed=True)
    if a.output_json:
        a.output_json.parent.mkdir(parents=True,exist_ok=True)
        a.output_json.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
