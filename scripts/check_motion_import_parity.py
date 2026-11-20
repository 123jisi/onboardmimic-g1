"""Opt-in offline comparison against trusted source trees; never sends data."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from onboardmimic.backends.sonic.motion_import import import_gmr, smpl_arrays


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--genmo',type=Path,required=True)
    p.add_argument('--sonic',type=Path,required=True)
    p.add_argument('--name',required=True)
    a=p.parse_args()
    # Read-only reference code. Importing the publisher never calls its main.
    import torch
    sys.path.insert(0,str(a.sonic))
    spec=importlib.util.spec_from_file_location('legacy_publisher',a.genmo/'sonic_extra/publish_smpl_params.py')
    publisher=importlib.util.module_from_spec(spec);spec.loader.exec_module(publisher)
    from gear_sonic.trl.utils import torch_transform
    torch_transform.human_joints_info=torch.load(a.sonic/'gear_sonic/data/human/human_joints_info.pkl',weights_only=False)
    params=publisher._resolve_smpl_dict(torch.load(a.genmo/'outputs/inference'/a.name/'smpl_params.pt',map_location='cpu',weights_only=True))
    pose=publisher._normalize_smpl_pose(params)
    old=dict(smpl_pose=pose.numpy(),smpl_joints=publisher._compute_smpl_joints_local(params,pose).numpy(),
             body_quat_w=publisher._compute_body_quat_sequence(publisher._extract_global_orient(params,len(pose))),
             joint_pos=np.stack([publisher._build_wrist_joint_pos_from_smpl(x.numpy()) for x in pose]),
             joint_vel=np.zeros((len(pose),29),dtype=np.float32))
    new=smpl_arrays({k:(v.detach().cpu().numpy() if isinstance(v,torch.Tensor) else v) for k,v in params.items()})
    errors={k:float(np.max(np.abs(new[k]-old[k]))) for k in old}
    if max(errors.values())>1e-5: raise AssertionError(errors)
    with tempfile.TemporaryDirectory() as folder:
        root=Path(folder);source=a.genmo/'outputs/gmr'/f'{a.name}.pkl'
        import_gmr(source,root/'new','mujoco')
        subprocess.run([sys.executable,str(a.genmo/'sonic_extra/convert_gmr_to_sonic.py'),str(source),'-o',str(root),'--motion-name','old','--input-joint-order','mujoco'],check=True,stdout=subprocess.DEVNULL)
        csv_errors={x.name:float(np.max(np.abs(np.loadtxt(x,delimiter=',',skiprows=1)-np.loadtxt(root/'old'/x.name,delimiter=',',skiprows=1)))) for x in (root/'new').glob('*.csv')}
        if max(csv_errors.values())>2e-6: raise AssertionError(csv_errors)
    print(json.dumps(dict(name=a.name,csv_max_abs=csv_errors,smpl_max_abs=errors),indent=2))


if __name__=='__main__': main()
