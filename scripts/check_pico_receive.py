"""Observe PICO receive files and CPU mapping only. Never publishes or controls."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from onboardmimic.backends.sonic.pico.pose_reader import PoseReader

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--status',type=Path,default=ROOT/'onboardmimic/inputs/pico/runs/live/status.json')
    p.add_argument('--duration',type=float,default=3.)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    if not math.isfinite(a.duration) or not 0<a.duration<=60:p.error('duration must be 0..60 seconds')
    reader=PoseReader(a.status);end=time.monotonic()+a.duration;error='';summary={}
    counts={k:set() for k in ('body','left_controller','right_controller')}
    while time.monotonic()<end:
        try:
            data=json.loads(a.status.read_text());reader.feed(data)
            summary={}
            for name in counts:
                s=data.get('streams',{}).get(name,{})
                if s.get('fresh') and type(s.get('source_timestamp_ns')) is int:counts[name].add(s['source_timestamp_ns'])
                summary[name]={k:s.get(k) for k in ('fresh','age_ms','unique_frame_hz','max_observed_gap_ms','error')}
            error=''
        except (OSError,ValueError,TypeError,KeyError,AttributeError) as exc:
            error=str(exc);reader.invalidate('接收文件无效')
        time.sleep(.02)
    snap=reader.snapshot()
    report=dict(status_path=str(a.status),duration_seconds=a.duration,streams=summary,
                observed_unique_frames={k:len(v) for k,v in counts.items()},
                sonic_mapping_ready=snap['ready'],controllers_ready=snap['controllers_ready'],
                controller_age_ms=snap['controller_age_ms'],reason=snap['reason'],error=error,
                motor_output=False,published_messages=0,
                note='Local receive/mapping freshness only; not end-to-end motion latency or motion acceptance.')
    text=json.dumps(report,ensure_ascii=False,indent=2)
    if a.output:a.output.write_text(text+'\n')
    print(text)

if __name__=='__main__':main()
