"""Validated NumPy SMPL cache; playback has no Torch/legacy-repo dependency."""
import hashlib
import json
from pathlib import Path

import numpy as np

from .csv_player import CSVClip


class SMPLClip:
    mode = 'smpl_playback'
    tilt_error = CSVClip.tilt_error

    def __init__(self, path):
        self.path = Path(path).resolve()
        if self.path.stat().st_size > 128*1024*1024:
            raise ValueError('SMPL cache exceeds 128 MiB')
        # Limit decoded archive size before allocating arrays.
        import zipfile
        with zipfile.ZipFile(self.path) as archive:
            if sum(x.file_size for x in archive.infolist()) > 256*1024*1024:
                raise ValueError('SMPL cache decoded size exceeds limit')
        with np.load(self.path, allow_pickle=False) as data:
            meta = json.loads(str(data['metadata']))
            if meta.get('version') != 'onboard-motion-1' or meta.get('fps') != 50 or meta.get('representation') != 'smpl':
                raise ValueError('Unsupported SMPL cache version/representation/fps')
            from .motion_import import conversion_fingerprint
            if meta.get('conversion_sha256') != conversion_fingerprint():
                raise ValueError('Stale SMPL cache; reimport original motion')
            self.arrays = {k:np.array(data[k],dtype=np.float32) for k in
                           ('smpl_pose','smpl_joints','body_quat_w','joint_pos','joint_vel')}
        self.frames = meta['frames']
        if type(self.frames) is not int or not 2 <= self.frames <= 30000:
            raise ValueError('Invalid SMPL frame count')
        for k,shape in dict(smpl_pose=(21,3),smpl_joints=(24,3),body_quat_w=(4,),joint_pos=(29,),joint_vel=(29,)).items():
            a = self.arrays[k]
            if a.shape != (self.frames,)+shape or not np.isfinite(a).all():
                raise ValueError('Invalid SMPL array: '+k)
            a.flags.writeable = False
        self.quat = self.arrays['body_quat_w']
        if np.any(np.abs(np.linalg.norm(self.quat,axis=1)-1) > .02):
            raise ValueError('Invalid SMPL root quaternion')
        self.q,self.dq = self.arrays['joint_pos'],self.arrays['joint_vel']
        self.fps,self.duration = 50.,self.frames/50.
        self.meta = dict(meta, name=self.path.name,duration_seconds=self.duration,
                         cache_sha256=hashlib.sha256(self.path.read_bytes()).hexdigest())

    def metadata(self):
        return dict(self.meta)


def main():
    import argparse
    p = argparse.ArgumentParser(description='Validate SMPL cache without control I/O')
    p.add_argument('path', type=Path)
    args = p.parse_args()
    print(json.dumps(SMPLClip(args.path).metadata(), indent=2))


if __name__ == '__main__': main()
