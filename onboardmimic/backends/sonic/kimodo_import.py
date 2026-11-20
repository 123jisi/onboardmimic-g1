"""Kimodo AMASS NPZ import, matching publish_smpl_kimodo.py body semantics.

Only 29-DoF body control. Optional face/finger/translation/shape fields are not
used by the legacy body's FK or wrist proxy and are not sent to a hand chain.
See docs/development/TELEOP_KIMODO_zh.md for source provenance and conventions.
"""
from pathlib import Path
import zipfile

import numpy as np

from .motion_import import checked, smpl_arrays, source_info, time_axes, write_smpl_cache


def load_amass(path, fps=None):
    """Read numeric body fields only; never deserialize object arrays."""
    path = Path(path)
    if path.stat().st_size > 128*1024*1024:
        raise ValueError('AMASS NPZ exceeds 128 MiB')
    with zipfile.ZipFile(path) as archive:
        if sum(x.file_size for x in archive.infolist()) > 256*1024*1024:
            raise ValueError('Decoded AMASS NPZ exceeds 256 MiB')
    with np.load(path, allow_pickle=False) as data:
        if not {'pose_body', 'root_orient'} <= set(data.files):
            raise ValueError('Expected AMASS pose_body + root_orient; convert raw Kimodo NPZ to AMASS on host first')
        pose = np.asarray(data['pose_body'], dtype=np.float32)
        if pose.ndim != 2 or pose.shape[1] != 63:
            raise ValueError('pose_body must be (T,63) axis-angle')
        n = len(pose)
        source_rates = {}
        for key in ('mocap_frame_rate', 'mocap_framerate', 'fps'):
            if key in data:
                rate = np.asarray(data[key])
                if rate.size != 1 or rate.dtype.kind not in 'fiu':
                    raise ValueError('Invalid numeric FPS metadata: '+key)
                source_rates[key] = float(rate.reshape(-1)[0])
        if fps is None:
            if not source_rates:
                raise ValueError('Missing source FPS metadata; provide --fps explicitly')
            rates = list(source_rates.values())
            if any(not np.isfinite(x) or abs(x-rates[0]) > 1e-6 for x in rates):
                raise ValueError('Conflicting or invalid source FPS metadata; provide actual --fps')
            fps = rates[0]
            fps_source = next(iter(source_rates))
        else:
            fps = float(fps)
            fps_source = 'explicit'
        time_axes(n, fps)  # bound frames/duration before FK allocations
        pose = checked(pose,(n,63),'pose_body')
        orient = checked(data['root_orient'],(n,3),'root_orient')
        ignored = sorted(set(data.files)-{'pose_body','root_orient'}-set(source_rates))
    return dict(body_pose=pose,global_orient=orient), fps, dict(
        fps_source=fps_source, file_fps=source_rates, ignored_fields=ignored)


def import_kimodo(path, output, fps=None):
    path = Path(path)
    params, fps, options = load_amass(path, fps)
    arrays = smpl_arrays(params, wrist_mapping='kimodo')
    meta = source_info(path,'kimodo_amass_npz',fps,len(arrays['smpl_pose']),
                       wrist_mapping='kimodo_interleaved_v1',
                       root_transform='Y_to_Z_then_remove_SMPL_base; matches legacy publisher',
                       **options)
    return write_smpl_cache(output, arrays, meta, wrist_mapping='kimodo')
