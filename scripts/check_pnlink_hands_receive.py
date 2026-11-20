"""Read-only right-finger receive probe. No device commands or actuator imports."""
import argparse
import json
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from onboardmimic.inputs.pnlink.state import read_state,fresh


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--status',type=Path,default=ROOT/'var/pnlink/status.json')
    p.add_argument('--duration',type=float,default=10)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    if not 0<a.duration<=600:p.error('duration must be in (0,600]')
    start=time.monotonic();seen=set();valid=invalid=0;intervals=[];last=None;errors=set()
    while time.monotonic()-start<a.duration:
        try:
            raw=read_state(a.status)
            if not fresh(raw):raise ValueError('PN body not fresh')
            h=raw.get('right_hand',{});key=raw['receiver_id'],h.get('frame_id')
            if key not in seen:
                seen.add(key)
                if h.get('valid') is not True:raise ValueError(h.get('error') or 'No complete right fingers')
                q=np.asarray(h['rotations_wxyz'],dtype=float)
                if q.shape!=(15,4) or not np.isfinite(q).all():raise ValueError('Invalid right quaternion shape')
                Rotation.from_quat(q[:,[1,2,3,0]]).as_rotvec()
                if last is not None:intervals.append((h['received_monotonic']-last)*1000)
                last=h['received_monotonic'];valid+=1
        except (OSError,ValueError,KeyError,TypeError) as exc:
            invalid+=1;errors.add(str(exc))
        time.sleep(.005)
    result=dict(duration_seconds=time.monotonic()-start,valid_right_hand_frames=valid,invalid_observations=invalid,
                optional_right_joints=15,receive_interval_max_ms=max(intervals) if intervals else None,
                errors=sorted(errors),sdk_calibration_requested=False,hand_calibration_performed=False,
                motor_output=False,note='Receive/schema only. Unworn suit cannot validate finger tracking.')
    if a.output:a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
