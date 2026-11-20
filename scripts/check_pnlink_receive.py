"""Read-only PN Link timing/mapping probe; never sends device or robot commands."""
import argparse
import json
from pathlib import Path
import resource
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from onboardmimic.inputs.pnlink.state import read_state, fresh


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--status',type=Path,default=ROOT/'var/pnlink/status.json')
    p.add_argument('--duration',type=float,default=10)
    p.add_argument('--map',action='store_true',help='Also measure CPU mapping; requires NumPy/SciPy')
    p.add_argument('--require-ready',action='store_true',help='Fail before timing if capture/body is not already fresh')
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    if not 0<a.duration<=3600:p.error('duration must be in (0,3600]')
    start=time.monotonic();cpu=time.process_time();last=None;last_received=None
    frames=invalid=0;generations=set();intervals=[];mapping=[];source=[];errors=set();data={}
    first_counter=last_counter=None;first_counter_time=last_counter_time=None
    if a.map:
        from onboardmimic.backends.sonic.pnlink.mapping import map_joints
    if a.require_ready:
        ready=read_state(a.status)
        if not fresh(ready):p.error('Start capture and wait for fresh body frames before the timed acceptance')
        if a.map:map_joints(ready['joints'])
    start=time.monotonic();cpu=time.process_time()
    while time.monotonic()-start<a.duration:
        try:
            data=read_state(a.status)
            if not fresh(data):invalid+=1
            key=(data.get('receiver_id'),data.get('frame_id'))
            if key!=last and fresh(data):
                if key[0] not in generations:last_received=None
                generations.add(key[0]);received=data['received_monotonic']
                if first_counter is None:first_counter=key[1];first_counter_time=received
                last_counter=key[1];last_counter_time=received
                if last_received is not None:intervals.append((received-last_received)*1000)
                last_received=received;last=key;frames+=1
                source.append(data.get('source_timestamp'))
                if a.map:
                    before=time.perf_counter();map_joints(data['joints']);mapping.append((time.perf_counter()-before)*1000)
        except (OSError,ValueError,KeyError,TypeError) as exc:
            invalid+=1;errors.add(str(exc))
        time.sleep(.005)
    def stats(values):
        values=sorted(values)
        return dict(count=len(values),p50=values[len(values)//2],p95=values[min(len(values)-1,int(len(values)*.95))],max=max(values)) if values else None
    elapsed=time.monotonic()-start
    result=dict(duration_seconds=elapsed,observed_frames=frames,observed_hz=frames/elapsed,
                invalid_reads=invalid,receiver_generations=len(generations),device=data.get('device'),
                endpoints=data.get('endpoints'),sdk_open=data.get('sdk_open'),connected=data.get('connected'),
                capture_state=data.get('capture_state'),command=data.get('command'),
                receiver_metrics=data.get('metrics'),
                sdk_avatar_hz=(last_counter-first_counter)/(last_counter_time-first_counter_time) if first_counter_time is not None and last_counter_time>first_counter_time and len(generations)==1 else None,
                receive_interval_ms=stats(intervals),mapping_ms=stats(mapping),
                sdk_timestamp_regressions=sum(b<a for a,b in zip(source,source[1:])),
                probe_cpu_seconds=time.process_time()-cpu,probe_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                errors=sorted(errors),robot_output_enabled=False,
                note='Observed receive/mapping timing, not end-to-end robot latency')
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
