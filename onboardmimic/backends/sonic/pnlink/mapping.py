"""PN named local wxyz -> SONIC; matches mocap_live_play_smplx + Kimodo publisher.

Root translation is intentionally absent in the reference SONIC payload. The FK
and wrist implementation/provenance are shared with the existing SMPL importer.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R
from ..motion_import import smpl_arrays
from ..pico.pose_mapping import mix as pico_mix
from ..pico.wrist_mapping import compute_wrist_joints

from onboardmimic.inputs.pnlink.model import BODY_NAMES, DERIVED_TERMINALS


def map_joints(joints):
    if not isinstance(joints, list) or not 20 <= len(joints) <= 128:
        raise ValueError('PN Link 身体关节不完整')
    found = {}
    for row in joints:
        name = row['name']
        if name not in BODY_NAMES:
            continue
        if name in found:
            raise ValueError('PN Link 身体关节名重复：' + name)
        q = np.asarray(row['rotation_wxyz'], dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or not .8 <= np.linalg.norm(q) <= 1.2:
            raise ValueError('PN Link 四元数无效：' + name)
        found[name] = R.from_quat(q[[1, 2, 3, 0]]).as_rotvec()
    missing = set(BODY_NAMES)-found.keys()-DERIVED_TERMINALS
    if missing:
        raise ValueError('PN Link 缺少身体关节：' + ', '.join(sorted(missing)))
    aa = np.asarray([found.get(name, np.zeros(3)) for name in BODY_NAMES], dtype=np.float32)
    arrays = smpl_arrays(dict(body_pose=np.repeat(aa[None, 1:], 2, axis=0),
                              global_orient=np.repeat(aa[None, 0], 2, axis=0)), wrist_mapping='kimodo')
    return {key: value[0].copy() for key, value in arrays.items()}


def mix(a, b, fraction):
    result = pico_mix(a, b, fraction)
    left, right = compute_wrist_joints(result['smpl_pose'].reshape(63))
    result['joint_pos'][[23, 25, 27]] = [left[0], -left[1], left[2]]
    result['joint_pos'][[24, 26, 28]] = right
    return result
