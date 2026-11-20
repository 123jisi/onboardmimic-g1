"""Receive-only snapshots -> PICO mode events and references; no sockets.

Lifecycle owns authorization, switching, feedback acknowledgement and publication.
"""
import math
import secrets
import numpy as np
from scipy.spatial.transform import Rotation as R
from .kinematics import ThreePointCalibration, UPPER_INDICES
from .pose_wire import pack_pose_message
from .recovery import PicoRecovery

MODES=('pose','planner','frozen','vr3pt')

class PicoSession(PicoRecovery):
    def __init__(self,reader,max_speed=5.):
        self.reader=reader;self.max_speed=max_speed;self.active=False
        self.mode='planner';self.parent='planner';self.released=False;self.previous=set()
        self.heading=0.;self.last=None;self.gait=0;self.calibration=ThreePointCalibration()
        self.paused=False;self.hold=None;self.last_fields=None;self.index=0
        self.receiver_id=None;self.error='';self.frozen=None
        self.reset_hands()
        self.reset_recovery()

    def reset_hands(self):
        self.hand_session=secrets.token_hex(16)
        self.hand_armed={'left':False,'right':False}
        self.hand_target={'left':None,'right':None}

    def hand_intent(self):
        if not self.active or self.recovering or not self.ready() or not self.released:
            return dict(left=None,right=None)
        if not self.paused:
            s=self.reader.snapshot()
            for side in ('left','right'):
                trigger=s['controllers'][side]['trigger']
                if trigger<=.5:self.hand_armed[side]=True
                self.hand_target[side]=trigger>.5 if self.hand_armed[side] else None
        return dict(self.hand_target)

    def ready(self,snapshot=None):
        s=self.reader.snapshot() if snapshot is None else snapshot
        return bool(s.get('ready') and s.get('controllers_ready') and s.get('raw_body') is not None)

    def revoke(self,reason=''):
        self.active=False;self.released=False;self.previous=set();self.paused=False
        self.hold=None;self.last_fields=None;self.error=reason;self.last=None
        self.reset_hands()
        self.reset_recovery()

    def enter(self,mode,q,new_session=False):
        if mode not in MODES:raise ValueError('未知 PICO 子模式')
        s=self.reader.snapshot()
        if not self.ready():raise ValueError('PICO 身体和双手柄尚未就绪')
        if new_session:
            self.reset_hands()
            self.calibration=ThreePointCalibration();self.released=False;self.previous=set()
            self.receiver_id=s.get('receiver_id');self.gait=0
        if new_session or mode=='vr3pt':self.calibration.capture(s['raw_body'],q)
        if mode=='frozen':self.frozen=np.asarray(q,dtype=float)[UPPER_INDICES].tolist()
        preserve_heading=not new_session and self.mode!='pose' and mode!='pose'
        self.mode=mode;self.active=True;self.error=''
        self.reset_recovery()
        if not preserve_heading:self.heading=0.
        self.last=None
        self.paused=False;self.hold=None;self.last_fields=None;self.index=0

    def status(self,snapshot=None):
        s=self.reader.snapshot() if snapshot is None else snapshot
        return dict(active=self.active,mode=self.mode if self.active else None,paused=self.paused,
                    controllers_ready=bool(s.get('controllers_ready')),
                    controller_age_ms=s.get('controller_age_ms',{}),
                    controller_errors=s.get('controller_errors',{}),
                    tracking=dict(body_ready=s.get('ready',False),body_reason=s.get('reason'),
                                  control_fault=s.get('control_fault',False),
                                  body_clock_fault=s.get('body_clock_fault',False),
                                  receiver_id=s.get('receiver_id'),streams=s.get('source_status',{})),
                    buttons_released=self.released,alignment_confirmed=self.active,
                    mapping_calibrated=self.active and self.calibration.neck is not None,
                    headset_calibration='operator_confirmed' if self.active else 'not_confirmed',
                    locomotion_mode=self.gait,error=self.recovery_status()['recovery_message'] or self.error,
                    **self.recovery_status())

    @staticmethod
    def buttons(s):
        l=s['controllers']['left'];r=s['controllers']['right']
        a,b,x,y=r['primary'],r['secondary'],l['primary'],l['secondary']
        return {k for k,v in dict(stop=a and b and x and y,ax=a and x,by=b and y,
                                  click=l['click'],ab=a and b,xy=x and y,menu=l['menu']).items() if v}

    def events(self):
        s=self.reader.snapshot()
        if not self.active:return None
        if self.recovering:return None
        if not self.ready() or s.get('receiver_id')!=self.receiver_id:return 'lost'
        buttons=self.buttons(s)
        if not self.released:
            # Both sticks must also be neutral before a new session can move.
            axes=s['controllers']['left']['axes']+s['controllers']['right']['axes']
            held=any(c[k] for c in s['controllers'].values() for k in ('primary','secondary','click','menu'))
            if not held and max(map(abs,axes))<.15:self.released=True
            self.previous=buttons
            return None
        rising=buttons-self.previous;self.previous=buttons
        if 'stop' in buttons:return 'stop'  # Takes precedence over every subset.
        if 'ax' in rising:return 'planner' if self.mode=='pose' else 'pose'
        if 'by' in rising:return 'frozen' if self.mode=='pose' else 'pose'
        if 'click' in rising:
            if self.mode in ('planner','frozen'):
                self.parent=self.mode;return 'vr3pt'
            if self.mode=='vr3pt':return self.parent
        if self.mode!='pose':
            if 'ab' in rising:self.gait=min(19,self.gait+1)
            elif 'xy' in rising:self.gait=max(0,self.gait-1)
        if self.mode=='pose':
            held='menu' in buttons
            if held and not self.paused:
                if self.last_fields is None:return 'lost'
                self.hold={k:v.copy() for k,v in self.last_fields.items()};self.paused=True
            elif not held and self.paused:
                fields=s['fields']
                # Resume only close to the frozen reference; no unbounded jump.
                a=R.from_rotvec(fields['smpl_pose'][-1]);b=R.from_rotvec(self.hold['smpl_pose'][-1])
                qa=R.from_quat(fields['body_quat_w'][-1][[1,2,3,0]])
                qb=R.from_quat(self.hold['body_quat_w'][-1][[1,2,3,0]])
                if np.max((a.inv()*b).magnitude())>.35 or (qa.inv()*qb).magnitude()>.15:return 'resume_rejected'
                self.paused=False;self.hold=None
        return None

    def reference(self,now):
        if self.active and self.recovering:return self.recovery_reference(now)
        s=self.reader.snapshot()
        if not self.active or not self.ready():return None
        dt=0. if self.last is None else min(.05,max(0.,now-self.last));self.last=now
        l=s['controllers']['left'];r=s['controllers']['right'];rx=r['axes'][0]
        if not self.released:rx=0.
        dyaw=-1.5*rx*dt
        if self.mode=='pose':
            fields={k:v.copy() for k,v in (self.hold if self.paused else s['fields']).items()}
            if not self.paused:
                vr=self.calibration.apply(s['raw_body'])
                fields.update(vr_position=vr[:,:3].reshape(-1),vr_orientation=vr[:,3:].reshape(-1),
                              heading_increment=np.array([dyaw],dtype=np.float32))
            else:fields['heading_increment']=np.zeros(1,dtype=np.float32)
            self.last_fields={k:v.copy() for k,v in fields.items()}
            self.remember_reference(fields)
            fields['frame_index']=np.arange(self.index,self.index+5,dtype=np.int64);self.index+=1
            fields['timestamp_monotonic']=np.array([now],dtype=np.float64)
            return pack_pose_message(fields,version=3)
        if abs(rx)>=.15:self.heading+=dyaw
        facing=[math.cos(self.heading),math.sin(self.heading),0.]
        lx,ly=l['axes'] if self.released else (0.,0.)
        raw=min(1.,math.hypot(lx,ly));mag=max(0.,(raw-.15)/.85)
        gait=self.gait if mag else 0
        speed=(.1+.5*mag if gait==1 else -1. if gait==2 else 1.5+3*mag if gait==3 else mag) if mag else 0.
        if speed>=0:speed=min(self.max_speed,speed)
        # Original manager's movement convention, relative to accumulated facing.
        scale=mag/(raw or 1.)
        movement=np.array([[-facing[1],facing[0]],[facing[0],facing[1]]])@np.array([-lx,ly])*scale
        packet=dict(mode=gait,movement_direction=[*movement,0.],facing_direction=facing,speed=speed,height=-1.)
        if self.mode=='frozen':packet.update(upper_body_position=self.frozen,upper_body_velocity=[0.]*17)
        if self.mode=='vr3pt':
            vr=self.calibration.apply(s['raw_body'])
            packet.update(vr_3pt_position=vr[:,:3].reshape(-1).tolist(),vr_3pt_orientation=vr[:,3:].reshape(-1).tolist())
        self.remember_reference(packet)
        return packet
