"""Read-only PICO -> SONIC SMPL mapping worker. Publication belongs to gateway."""
from collections import deque
import json
from pathlib import Path
import threading
import time
import sys
import numpy as np
from onboardmimic.backends.sonic.pico.pose_mapping import map_body,mix
from onboardmimic.backends.sonic.pico.pose_wire import pack_pose_message
from onboardmimic.core.runtime_timing import wait_event
from onboardmimic.inputs.pico.receiver import controller

class PoseReader:
    def __init__(self,path,clock=time.monotonic):
        self.path=Path(path);self.clock=clock;self.lock=threading.RLock();self.stop=threading.Event()
        self.prev=None;self.next_stamp=None;self.frames=deque(maxlen=5);self.index=0
        self.updated=None;self.packet=None;self.reason='等待 PICO 全身数据';self.regressed=False
        self.fields=None;self.raw_body=None;self.receiver_id=None
        self.controls={};self.control_fault=False;self.control_errors={};self.source_status={}
        self.thread=threading.Thread(target=self.run,daemon=True)
    def invalidate(self,reason):
        with self.lock:
            self.packet=None;self.fields=None;self.raw_body=None;self.frames.clear();self.prev=None;self.next_stamp=None;self.reason=reason
    def feed(self,data,wall=None):
        wall=time.time() if wall is None else wall;now=self.clock()
        with self.lock:
            generation=data.get('receiver_id')
            if generation and generation!=self.receiver_id:
                self.invalidate('接收器已重启');self.controls.clear();self.control_fault=False;self.control_errors.clear();self.regressed=False
                self.receiver_id=generation
            self.source_status={name:{key:stream.get(key) for key in
                ('fresh','age_ms','source_timestamp_ns','error','max_observed_gap_ms')}
                for name in ('body','left_controller','right_controller')
                for stream in (data.get('streams',{}).get(name,{}),)}
            self.feed_controls(data,wall,now)
            b=data.get('streams',{}).get('body',{});age=wall-float(data.get('updated_at_unix',0))
            stamp=b.get('source_timestamp_ns');poses=b.get('payload')
            if (data.get('data_source')!='sdk' or data.get('receiver_running') is not True or
                b.get('fresh') is not True or not 0<=age<=.25 or
                type(b.get('age_ms')) not in (int,float) or not 0<=b['age_ms']+age*1000<=250 or
                type(stamp) is not int or stamp<=0):
                self.invalidate('PICO 身体数据已断流');return
            if self.regressed:return
            if self.prev and stamp<self.prev[0]:
                self.regressed=True;self.invalidate('PICO 时间戳回退，请重启接收适配器');return
            if self.prev and stamp==self.prev[0]:return
            mapped=map_body(poses)
            self.raw_body=np.asarray(poses,dtype=float).copy()
            if self.prev and stamp-self.prev[0]>250000000:self.invalidate('PICO 数据间隔过大')
            if self.prev is None:
                self.prev=(stamp,mapped);self.next_stamp=stamp;return
            old_stamp,old=self.prev
            while self.next_stamp<=stamp:
                if self.next_stamp>=old_stamp:
                    a=(self.next_stamp-old_stamp)/(stamp-old_stamp)
                    row=mix(old,mapped,a);row['frame_index']=np.int64(self.index);self.index+=1;self.frames.append(row)
                self.next_stamp+=20000000
            self.prev=(stamp,mapped);self.updated=now
            if len(self.frames)==5:
                fields={k:np.stack([r[k] for r in self.frames]) for k in self.frames[0]}
                fields['timestamp_monotonic']=np.array([now],dtype=np.float64)
                self.fields=fields
                self.packet=pack_pose_message(fields,version=3);self.reason='SONIC SMPL · 50 Hz · 5 帧窗口'
    def feed_controls(self,data,wall,now):
        age=wall-float(data.get('updated_at_unix',0))
        for side in ('left','right'):
            b=data.get('streams',{}).get(side+'_controller',{})
            stamp=b.get('source_timestamp_ns');a=b.get('age_ms')
            valid=(data.get('data_source')=='sdk' and data.get('receiver_running') is True and
                   b.get('fresh') is True and 0<=age<=.25 and type(a) in (int,float) and
                   0<=a+age*1000<=250 and type(stamp) is int and stamp>0)
            old=self.controls.get(side)
            if old and valid and stamp<old['stamp']:
                self.control_fault=True;self.control_errors[side]='时间戳回退，需重启接收器'
            if not valid:
                if old:old['valid']=False
                self.control_errors[side]=b.get('error') or '手柄数据过期或不可用'
                continue
            payload=controller(b.get('payload'))
            if not old or stamp!=old['stamp']:
                self.controls[side]=dict(stamp=stamp,at=now,payload=payload,valid=True)
                if not self.control_fault:self.control_errors.pop(side,None)
            # A duplicate after invalidation cannot revive the old sample.
    def snapshot(self):
        with self.lock:
            fresh=self.updated is not None and 0<=self.clock()-self.updated<=.25
            controls_ready=(not self.control_fault and len(self.controls)==2 and
                            all(c['valid'] and 0<=self.clock()-c['at']<=.25 for c in self.controls.values()))
            errors=dict(self.control_errors)
            for side in ('left','right'):
                c=self.controls.get(side)
                if c is None:errors.setdefault(side,'等待手柄数据')
                elif not 0<=self.clock()-c['at']<=.25:errors.setdefault(side,'手柄时间戳超过 250 ms 未更新')
            return dict(ready=bool(fresh and self.packet),reason=self.reason if fresh else '等待新鲜 PICO 身体数据',packet=self.packet if fresh else None,
                        fields=self.fields if fresh else None,raw_body=self.raw_body if fresh else None,
                        controllers_ready=controls_ready,controllers={k:dict(v['payload']) for k,v in self.controls.items()} if controls_ready else {},
                        receiver_id=self.receiver_id,
                        body_age_ms=None if self.updated is None else round(max(0,self.clock()-self.updated)*1000,2),
                        control_fault=self.control_fault,body_clock_fault=self.regressed,controller_errors=errors,
                        source_status={k:dict(v) for k,v in self.source_status.items()},
                        controller_age_ms={k:round(max(0,self.clock()-v['at'])*1000,2) for k,v in self.controls.items()})
    def run(self):
        while not self.stop.is_set():
            try:self.feed(json.loads(self.path.read_text()))
            except (OSError,ValueError,TypeError,KeyError,AttributeError):self.invalidate('PICO 数据读取或映射失败')
            wait_event(self.stop, .01)
