"""Actual hand feedback + isolated continuous intent; always observe-only.

Uses a random localhost port. The child cannot create a DDS motor publisher.
No live gateway, controller, calibration or device commands are used.
"""
import argparse
import json
from pathlib import Path
import secrets
import subprocess
import tempfile
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface',required=True)
    p.add_argument('--duration',type=float,default=60)
    p.add_argument('--output',type=Path)
    p.add_argument('--hand',choices=('left','right','both'),default='right')
    a=p.parse_args()
    selected=('left','right') if a.hand=='both' else (a.hand,)
    if not 1<=a.duration<=300:p.error('duration must be 1..300 seconds')
    import zmq
    folder=Path(__file__).resolve().parents[1]/'onboardmimic/robots/unitree_g1/hands'
    ctx=zmq.Context();pub=ctx.socket(zmq.PUB);pub.setsockopt(zmq.LINGER,0)
    port=pub.bind_to_random_port('tcp://127.0.0.1')
    counts=dict(reports=0,active_reports=0,false_future_feedback=0,not_ready=0,intent_lost=0)
    errors=[];last=None;maximum_age=0.;result={}
    try:
        with tempfile.TemporaryDirectory(prefix='hand-feedback-timing-') as tmp:
            status=Path(tmp)/'status.json'
            child=subprocess.Popen([str(folder/'build/pico_hand_bridge'),'--observe-only','--interface',a.interface,
                '--status',str(status),'--port',str(port),'--duration',str(a.duration+15)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            try:
                until=time.monotonic()+8
                while not status.exists():
                    if child.poll() is not None:raise RuntimeError(child.communicate()[0])
                    if time.monotonic()>until:raise RuntimeError('Observe-only worker not ready')
                    time.sleep(.02)
                s=json.loads(status.read_text());bridge=s['bridge_id'];session=secrets.token_hex(16)
                begin=time.monotonic();sequence=0;active_seen=False
                while time.monotonic()-begin<a.duration+5.5:
                    elapsed=time.monotonic()-begin;active=elapsed>=5.5
                    sequence+=1
                    goal=[.15,.1875,.25,.25,.25,.25] if int(elapsed//2)%2 else [0.]*6
                    packet=dict(protocol=2,source='pnlink',bridge_id=bridge,session=session if active else '',
                        sequence=sequence,at_ns=time.monotonic_ns(),active=active,left=goal if active and 'left' in selected else None,right=goal if active and 'right' in selected else None)
                    pub.send(b'pico_hands'+json.dumps(packet).encode())
                    s=json.loads(status.read_text())
                    assert s['observe_only'] is True and all(v['commands_sent']==0 for v in s['sides'].values())
                    if s['updated_monotonic_ns']!=last:
                        last=s['updated_monotonic_ns'];counts['reports']+=1
                        for side in selected:
                            fb=s['sides'][side]
                            if fb['samples']>=10:
                                if fb['age_ms'] is None and not fb['ready']:counts['false_future_feedback']+=1
                                if fb['age_ms'] is not None:maximum_age=max(maximum_age,fb['age_ms'])
                            if active and s['active']:
                                if not fb['ready']:counts['not_ready']+=1
                                if not fb['commanding']:errors.append(side+': '+(fb.get('error') or 'Not commanding'))
                        if active and s['active']:
                            active_seen=True;counts['active_reports']+=1
                        elif active_seen:counts['intent_lost']+=1
                    time.sleep(.01)
                result=dict(passed=bool(counts['active_reports']>0 and not counts['false_future_feedback'] and not counts['not_ready']
                    and not counts['intent_lost'] and not errors),duration_seconds=a.duration,selected_sides=list(selected),**counts,
                    max_feedback_age_ms=maximum_age,errors=sorted(set(errors)),observe_only=True,motor_output=False,commands_sent=0)
            finally:
                child.terminate();child.communicate(timeout=5)
        if a.output:a.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
        if not result['passed']:raise SystemExit(1)
    finally:
        pub.close(0);ctx.term()

if __name__=='__main__':main()
