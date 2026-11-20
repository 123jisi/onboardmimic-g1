"""Observe real PICO buttons with the runtime STOPPED. No control POSTs.

Coverage records received inputs, not verified robot behavior or sensor freshness.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from urllib.request import urlopen
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from onboardmimic.inputs.pico.receiver import controller

REQUIRED=('a','b','x','y','left_click','left_menu','ax','by','ab','xy','abxy',
          'left_axis_left','left_axis_right','left_axis_up','left_axis_down',
          'right_axis_left','right_axis_right','right_axis_up','right_axis_down')


def stopped():
    with urlopen('http://127.0.0.1:8769/api/controller',timeout=2) as r:s=json.load(r)
    if s.get('state')!='stopped' or s.get('pids')!=[]:
        raise RuntimeError('Require runtime stopped with pids=[]; observation aborted')


def inputs(l,r):
    a,b,x,y=r['primary'],r['secondary'],l['primary'],l['secondary']
    pressed={k for k,v in dict(a=a,b=b,x=x,y=y,ax=a and x,by=b and y,ab=a and b,xy=x and y,
        abxy=a and b and x and y,left_click=l['click'],right_click=r['click'],
        left_menu=l['menu'],right_menu=r['menu']).items() if v}
    for side,c in (('left',l),('right',r)):
        for label,value in (('left',-c['axes'][0]),('right',c['axes'][0]),
                            ('down',-c['axes'][1]),('up',c['axes'][1])):
            if value>.3:pressed.add(side+'_axis_'+label)
        for key in ('trigger','grip'):
            if c[key]>.5:pressed.add(side+'_'+key)
    return pressed


class Observation:
    def __init__(self):
        self.observed=dict.fromkeys(REQUIRED,0);self.releases=dict.fromkeys(REQUIRED,0)
        self.previous=None;self.receiver_id=None;self.stamps={};self.last_change={}
        self.fresh_samples=0;self.stale_samples=0;self.neutral_samples=0
        self.stream_losses=dict.fromkeys(('body','left_controller','right_controller'),0)
        self.was_fresh={};self.errors={};self.last_error='';self.generations=0

    def sample(self,data,wall,now):
        try:
            age=wall-float(data.get('updated_at_unix',0))
            if (data.get('data_source')!='sdk' or data.get('receiver_running') is not True or
                    not math.isfinite(age) or not 0<=age<=.25):
                raise ValueError('receiver file stale or not live SDK')
            rid=data.get('receiver_id')
            if not isinstance(rid,str) or not rid:raise ValueError('missing receiver_id')
            if rid!=self.receiver_id:
                self.receiver_id=rid;self.stamps={};self.last_change={};self.previous=None;self.generations+=1
            streams=data.get('streams',{});valid={}
            for name in self.stream_losses:
                s=streams.get(name,{})
                stamp=s.get('source_timestamp_ns');ms=s.get('age_ms')
                good=(s.get('fresh') is True and type(ms) in (int,float) and
                      math.isfinite(ms) and 0<=ms+age*1000<=250 and type(stamp) is int and stamp>0)
                old=self.stamps.get(name)
                if good and old is not None and stamp<old:raise ValueError(name+' timestamp regressed')
                if good and stamp!=old:self.stamps[name]=stamp;self.last_change[name]=now
                good=good and now-self.last_change.get(name,-math.inf)<=.25
                valid[name]=good
                if self.was_fresh.get(name) is True and not good:self.stream_losses[name]+=1
                self.was_fresh[name]=good
            if not all(valid.values()):
                raise ValueError('stale streams: '+','.join(k for k,v in valid.items() if not v))
            l=controller(streams['left_controller']['payload']);r=controller(streams['right_controller']['payload'])
            pressed=inputs(l,r);self.fresh_samples+=1;self.last_error=''
            # Require a known neutral sample before counting subsequent edges.
            neutral=(not pressed and all(abs(v)<.15 for c in (l,r) for v in c['axes'])
                     and all(not c[k] for c in (l,r) for k in ('primary','secondary','click','menu')))
            if neutral:self.neutral_samples+=1
            if self.previous is None:
                if neutral:self.previous=set();return 'neutral baseline ready'
                return None
            rising=pressed-self.previous;falling=self.previous-pressed
            for k in rising:self.observed[k]=self.observed.get(k,0)+1
            for k in falling:self.releases[k]=self.releases.get(k,0)+1
            self.previous=pressed
            if rising or falling:
                return 'pressed='+','.join(sorted(rising))+' released='+','.join(sorted(falling))
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            return self.invalid(str(exc))
        return None

    def invalid(self,message):
        self.stale_samples+=1;self.previous=None
        self.errors[message]=self.errors.get(message,0)+1
        if message!=self.last_error:self.last_error=message;return message
        return None

    def report(self,duration):
        missing=[k for k in REQUIRED if not self.observed[k] or not self.releases[k]]
        return dict(recorded_at_utc=datetime.now(timezone.utc).isoformat(),duration_seconds=duration,
            observed=self.observed,releases=self.releases,missing=missing,
            input_coverage_complete=not missing and self.previous==set(),
            neutral_samples=self.neutral_samples,ended_neutral=self.previous==set(),
            fresh_samples=self.fresh_samples,stale_samples=self.stale_samples,
            stream_loss_events=self.stream_losses,receiver_generations=self.generations,errors=self.errors,
            runtime_state='stopped',pids=[],motor_output=False,published_messages=0,
            note='Received input coverage only. Operator must confirm physical labels/directions. '
                 'Does not prove controller disconnect behavior or live SONIC mode/motion acceptance.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--duration',type=int,default=60)
    p.add_argument('--output',type=Path);p.add_argument('--watch',action='store_true')
    a=p.parse_args()
    if not 1<=a.duration<=120:p.error('duration must be 1..120 seconds')
    stopped();start=time.monotonic();last_check=start;probe=Observation()
    path=ROOT/'onboardmimic/inputs/pico/runs/live/status.json'
    if a.watch:print('READY: release all buttons, center both sticks, then test inputs. Runtime must remain STOPPED.',flush=True)
    while time.monotonic()-start<a.duration:
        now=time.monotonic()
        if now-last_check>=1:stopped();last_check=now
        try:message=probe.sample(json.loads(path.read_text()),time.time(),now)
        except (OSError,ValueError) as exc:message=probe.invalid(str(exc))
        if a.watch and message:print(f'{now-start:6.1f}s {message}',flush=True)
        time.sleep(.01)
    stopped();report=probe.report(a.duration);text=json.dumps(report,ensure_ascii=False,indent=2)
    if a.output:a.output.write_text(text+'\n')
    print(text,flush=True)

if __name__=='__main__':main()
