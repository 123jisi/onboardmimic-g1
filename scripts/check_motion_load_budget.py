#!/usr/bin/env python3
"""Offline load/packing resource probe. No sockets, feedback, SONIC or motor I/O."""
import argparse
import json
from pathlib import Path
import resource
import sys
import threading
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from onboardmimic.backends.sonic.adapter import SonicBackend
from onboardmimic.backends.sonic.csv_player import CSVPlayer


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-root',required=True,type=Path)
    parser.add_argument('motions',nargs='+')
    args=parser.parse_args()
    backend=SonicBackend()
    player=CSVPlayer()
    gaps=[];failures=[];stop=threading.Event()
    def sample():
        previous=time.monotonic()
        while not stop.wait(.02):
            now=time.monotonic();gaps.append(now-previous);previous=now
            try:
                if player.clip:
                    if player.state not in ('playing','paused'):
                        player.start(now);player.confirm(now)
                    player.packet(now)
                    if player.error:failures.append(player.error)
            except Exception as exc:failures.append(str(exc))
    thread=threading.Thread(target=sample,daemon=True);thread.start()
    results=[]
    try:
        for name in args.motions:
            started=time.monotonic()
            clip=backend.prepare_clip(args.motion_root,dict(motion=name,fps=50,
                joint_order='sonic_reference',root_body_index=0,trusted_cli=True),lambda:False)
            # Clip installation is exercised by lifecycle tests. This probe only
            # publishes the first reference while loading later targets.
            if player.clip is None:player.load(clip)
            results.append(dict(motion=name,load_seconds=time.monotonic()-started,
                                frames=clip.frames,representation=clip.mode))
    finally:
        stop.set();thread.join(2)
    report=dict(scope='offline loader + Python 50Hz packing only; no SONIC/feedback/motor I/O',
        results=results,samples=len(gaps),max_sampler_gap_seconds=max(gaps,default=0),
        child_peak_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
        parent_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        errors=failures)
    print(json.dumps(report,indent=2))
    if failures or max(gaps,default=0) >= player.producer_timeout:
        raise SystemExit(1)

if __name__=='__main__':main()
