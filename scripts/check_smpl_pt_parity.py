"""Compare a PT cache with the original SONIC publisher, without control I/O."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from onboardmimic.backends.sonic.motion_import import import_pt
from onboardmimic.backends.sonic.smpl_player import SMPLClip


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sonic', type=Path, required=True, help='Trusted original GR00T source tree')
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--fps', type=float, required=True, help='Actual source FPS')
    p.add_argument('--group', choices=['auto', 'body_params_global', 'body_params_incam'], default='auto')
    p.add_argument('--cache', type=Path, help='Existing cache, e.g. copied from G1; otherwise import temporarily')
    p.add_argument('--output-json', type=Path)
    a = p.parse_args()
    import torch
    from scipy.spatial.transform import Rotation as R, Slerp
    sys.path.insert(0, str(a.sonic.resolve()))
    source = a.sonic/'gear_sonic/scripts/publish_smpl_params.py'
    spec = importlib.util.spec_from_file_location('legacy_pt_publisher', source)
    publisher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(publisher)  # Does not call publisher.main or bind ZMQ.
    from gear_sonic.trl.utils import torch_transform
    torch_transform.human_joints_info = torch.load(
        a.sonic/'gear_sonic/data/human/human_joints_info.pkl', weights_only=False)
    raw = torch.load(a.input, map_location='cpu', weights_only=True)
    params = publisher._resolve_smpl_dict(raw, preferred_group=a.group)
    pose = publisher._normalize_smpl_pose(params)
    old = dict(smpl_pose=pose.numpy(),
               smpl_joints=publisher._compute_smpl_joints_local(params, pose).numpy(),
               body_quat_w=publisher._compute_body_quat_sequence(publisher._extract_global_orient(params, len(pose))),
               joint_pos=np.stack([publisher._build_wrist_joint_pos_from_smpl(x.numpy()) for x in pose]),
               joint_vel=np.zeros((len(pose), 29), dtype=np.float32))
    source_hash = hashlib.sha256(a.input.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory() as tmp:
        path = a.cache or Path(tmp)/'cache.npz'
        if a.cache is None:
            import_pt(a.input, path, a.fps, a.group)
        clip = SMPLClip(path)
        if (clip.meta.get('source_sha256') != source_hash or clip.meta.get('source_fps') != a.fps
                or clip.meta.get('options', {}).get('group') != a.group):
            raise ValueError('Cache source/FPS/group does not match comparison input')
        with np.load(path, allow_pickle=False) as z:
            errors = {k:float(np.max(np.abs(z['source_'+k]-v))) for k,v in old.items()}
            # SciPy versions may choose opposite initial quaternion signs.
            # q and -q encode exactly the same rotation; align per frame before
            # comparing components, while checking angular error below as well.
            cache_quat = z['source_body_quat_w'].copy()
            opposite = np.sum(cache_quat*old['body_quat_w'], axis=1) < 0
            cache_quat[opposite] *= -1
            errors['body_quat_w'] = float(abs(cache_quat-old['body_quat_w']).max())
        ti = np.arange(len(pose))/a.fps
        to = np.linspace(0, ti[-1], clip.frames)
        expected_pose = np.stack([Slerp(ti, R.from_rotvec(old['smpl_pose'][:,j]))(to).as_rotvec()
                                  for j in range(21)], axis=1)
        expected_joints = np.stack([np.interp(to, ti, c) for c in old['smpl_joints'].reshape(len(pose),-1).T],
                                   axis=-1).reshape(clip.frames,24,3)
        expected_quat = Slerp(ti, R.from_quat(old['body_quat_w'][:,[1,2,3,0]]))(to)
        actual_quat = R.from_quat(clip.quat[:,[1,2,3,0]])
        expected_wrist = np.stack([publisher._build_wrist_joint_pos_from_smpl(x.astype(np.float32)) for x in expected_pose])
        resampled = dict(smpl_pose=float(abs(expected_pose-clip.arrays['smpl_pose']).max()),
                         smpl_joints=float(abs(expected_joints-clip.arrays['smpl_joints']).max()),
                         root_rotation_radians=float((expected_quat*actual_quat.inv()).magnitude().max()),
                         joint_pos=float(abs(expected_wrist-clip.q).max()))
        if max([*errors.values(), *resampled.values()]) > 1e-5:
            raise AssertionError(dict(original=errors, resampled=resampled))
    result = dict(input_sha256=source_hash, publisher_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  source_frames=len(pose), source_fps=a.fps, output_frames=clip.frames,
                  output_fps=50, playback_duration=clip.duration, group=a.group,
                  original_frame_max_abs=errors, resampled_max_abs=resampled,
                  equivalent_quaternion_sign_frames=int(opposite.sum()),
                  tolerance=1e-5, passed=True, controller_started=False)
    if a.output_json:
        a.output_json.parent.mkdir(parents=True, exist_ok=True)
        a.output_json.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
