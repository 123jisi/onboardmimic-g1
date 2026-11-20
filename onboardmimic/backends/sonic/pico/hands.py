"""PICO binary hand intent over the gateway's existing localhost publisher.

The G1 hand worker translates this intent to its installed BrainCo DDS driver.
No hardware imports, sockets, or actuator output here.
"""
import json
from pathlib import Path
import time

TOPIC=b'pico_hands'


class PicoHandLink:
    def __init__(self,path,clock_ns=time.monotonic_ns):
        self.path=Path(path);self.clock_ns=clock_ns;self.sequence=0

    def status(self):
        try:
            s=json.loads(self.path.read_text());age=(self.clock_ns()-s['updated_monotonic_ns'])/1e6
            if s.get('protocol')!=1 or s.get('running') is not True or not 0<=age<=500:
                raise ValueError('stale hand worker')
            if not isinstance(s.get('bridge_id'),str) or len(s['bridge_id'])!=32:
                raise ValueError('invalid hand worker identity')
            return dict(s,enabled=True,fresh=True,age_ms=round(age,2))
        except (OSError,ValueError,KeyError,TypeError):
            return dict(enabled=True,fresh=False,error='等待手部适配器')

    def message(self,gateway):
        status=self.status()
        if not status.get('fresh'):return None
        p=gateway.pico_session
        active=bool(gateway.phase=='active' and gateway.pico_owned() and
                    not gateway.mode_pending and not gateway.task_active() and
                    not getattr(p,'recovering',False) and p.ready())
        intent=p.hand_intent() if active else dict(left=None,right=None)
        self.sequence+=1
        data=dict(protocol=1,bridge_id=status['bridge_id'],session=p.hand_session if active else '',
                  sequence=self.sequence,at_ns=self.clock_ns(),active=active,
                  left=intent['left'],right=intent['right'])
        return TOPIC+json.dumps(data,separators=(',',':'),allow_nan=False).encode()
