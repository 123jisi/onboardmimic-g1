"""Real DDS feedback + isolated synthetic intent; hard-coded observe-only worker.

Never connects to the running gateway, creates a DDS command publisher or moves
hands. All synthetic packets go to a random localhost port and a temporary ID.
"""
import argparse
import json
from pathlib import Path
import secrets
import subprocess
import tempfile
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--interface',required=True);p.add_argument('--output',type=Path);p.add_argument('--require-both',action='store_true');a=p.parse_args()
    import zmq
    root=Path(__file__).resolve().parent
    ctx=zmq.Context();pub=ctx.socket(zmq.PUB);pub.setsockopt(zmq.LINGER,0);port=pub.bind_to_random_port('tcp://127.0.0.1')
    report={}
    try:
        with tempfile.TemporaryDirectory(prefix='pico-hands-observe-') as tmp:
            status=Path(tmp)/'status.json'
            child=subprocess.Popen([str(root/'build/pico_hand_bridge'),'--observe-only','--interface',a.interface,'--status',str(status),'--port',str(port),'--duration','25'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            try:
                until=time.monotonic()+8
                while not status.exists():
                    if child.poll() is not None:raise RuntimeError(child.communicate()[0])
                    if time.monotonic()>until:raise RuntimeError('Observe-only worker not ready')
                    time.sleep(.05)
                def snapshot():return json.loads(status.read_text())
                bridge=snapshot()['bridge_id'];seq=0;session=secrets.token_hex(16)
                def pump(seconds,active,sid=session,age=0,protocol=1,finish_open=False,hand='right'):
                    nonlocal seq
                    end=time.monotonic()+seconds
                    while time.monotonic()<end:
                        seq+=1
                        data=dict(protocol=protocol,bridge_id=bridge,session=sid if active or finish_open else '',sequence=seq,at_ns=time.monotonic_ns()-round(age*1e9),active=active,left=False,right=True)
                        if finish_open:data.update(finish_open=True,finish_sides=['left','right'] if hand=='both' else [hand])
                        if protocol==2:data.update(source='pnlink',
                            left=[.05,.07,.12,.02,.03,.04] if active and hand in ('left','both') else None,
                            right=[.15,.1875,.25,.25,.25,.25] if active and hand in ('right','both') else None)
                        pub.send(b'pico_hands'+json.dumps(data).encode());time.sleep(.02)
                    return snapshot()
                pump(5.5,False)
                active=pump(.8,True)
                assert active['observe_only'] and active['active'],active
                # Missing left feedback does not prevent validating a fresh right hand.
                for side in active['sides'].values():assert side['commands_sent']==0
                time.sleep(.45);expired=snapshot();assert not expired['active'],expired
                pump(.25,False);old=pump(.4,True);assert not old['active'],old
                second=secrets.token_hex(16);new=pump(.6,True,sid=second);assert new['active'],new
                stale=pump(.5,True,sid=second,age=.5);assert not stale['active'],stale
                pump(.3,False)
                third=secrets.token_hex(16)
                continuous=pump(.8,True,sid=third,protocol=2)
                assert continuous['active'] and 2 in continuous['intent_protocols'],continuous
                assert continuous['sides']['right']['commanding'],continuous
                assert not continuous['sides']['left']['commanding'],continuous
                assert all(v['commands_sent']==0 for v in continuous['sides'].values())
                ending=pump(.5,False,sid=third,protocol=2,finish_open=True)
                assert ending['finish_open_supported'] and ending['sides']['right']['finish_session']==third,ending
                assert ending['sides']['right']['finish_stage'] in ('opening','succeeded'),ending
                assert all(v['commands_sent']==0 for v in ending['sides'].values())
                replay=pump(.4,True,sid=third,protocol=2);assert not replay['active'],replay
                bilateral = False
                if a.require_both:
                    assert all(v['ready'] for v in snapshot()['sides'].values()),snapshot()
                    for hand in ('left','both'):
                        pump(.2,False,protocol=2)
                        sid=secrets.token_hex(16)
                        live=pump(.8,True,sid=sid,protocol=2,hand=hand)
                        assert live['active'] and live['sides']['left']['commanding'],live
                        assert live['sides']['right']['commanding']==(hand=='both'),live
                        if hand=='both':assert live['sides']['left']['target']!=live['sides']['right']['target'],live
                        done=pump(.5,False,sid=sid,protocol=2,finish_open=True,hand=hand)
                        for side in (('left','right') if hand=='both' else ('left',)):
                            assert done['sides'][side]['finish_session']==sid,done
                            assert done['sides'][side]['finish_stage'] in ('opening','succeeded'),done
                        assert all(v['commands_sent']==0 for v in done['sides'].values())
                        replay=pump(.3,True,sid=sid,protocol=2,hand=hand);assert not replay['active'],replay
                    bilateral=True
                report=dict(passed=True,observe_only=True,motor_output=False,commands_sent=0,
                    active_intent_accepted=True,bilateral_and_left_only_accepted=bilateral,continuous_right_only_accepted=True,finish_open_accepted=True,finish_session_replay_rejected=True,timeout_revoked=True,same_session_rejected_after_timeout=True,
                    new_session_accepted=True,stale_packets_rejected=True,
                    feedback={k:dict(ready=v['ready'],samples=v['samples'],age_ms=v['age_ms']) for k,v in active['sides'].items()})
            finally:
                child.terminate()
                try:child.communicate(timeout=5)
                except subprocess.TimeoutExpired:child.kill();child.communicate();raise RuntimeError('Observe-only worker did not exit')
        if a.output:a.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:pub.close(0);ctx.term()


if __name__=='__main__':main()
