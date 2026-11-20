"""Read-only mmap view of a clip validated in an isolated loader process."""
from pathlib import Path
import numpy as np
import weakref
from .csv_player import CSVClip
from .motion_library import run_worker


class PreparedClip:
    tilt_error = CSVClip.tilt_error

    def __init__(self, result, area):
        self.area = area
        self._cleanup = weakref.finalize(self, area.cleanup)
        self.mode = result['mode']
        self.meta = result['metadata']
        self.path = Path(result['motion'])
        self.frames = self.meta['frames']
        self.fps = self.meta['fps']
        self.duration = self.meta['duration_seconds']
        self.arrays = {name:np.load(Path(area.name)/(name+'.npy'), mmap_mode='r', allow_pickle=False)
                       for name in result['arrays']}
        self.q, self.dq, self.quat = (self.arrays.pop(k) for k in ('q','dq','quat'))

    def metadata(self):
        return dict(self.meta)


def prepare_clip(root, spec, cancelled):
    result, area = run_worker(dict(action='load', root=str(root), spec=spec), cancelled=cancelled)
    try: return PreparedClip(result, area)
    except BaseException:
        area.cleanup()
        raise
