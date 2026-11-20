"""Bounded, gateway-owned tracking hold. Never treats cached input as live."""
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation as R


class PicoRecovery:
    recovery_seconds = 3.
    recovery_stable_seconds = .2

    def reset_recovery(self):
        self.recovery_since = None
        self.recovery_fresh_since = None
        self.recovery_checked_at = 0.
        self.recovery_reason = ''
        self.cached_reference = None
        self.held_reference = None

    @property
    def recovering(self):
        return self.recovery_since is not None

    def remember_reference(self, reference):
        self.cached_reference = deepcopy(reference)

    def begin_tracking_hold(self, now):
        self.recovery_since = now
        self.recovery_fresh_since = None
        self.held_reference = deepcopy(self.cached_reference)
        if self.mode == 'pose':
            # Flatten the temporal window: repeating an old moving window would
            # keep replaying motion rather than hold its final posture.
            for key in ('smpl_pose','smpl_joints','body_quat_w','joint_pos','joint_vel'):
                value = self.held_reference[key]
                self.held_reference[key] = np.repeat(value[-1:],len(value),axis=0)
            self.held_reference['joint_vel'][:] = 0
            self.held_reference['heading_increment'] = np.zeros(1,dtype=np.float32)
        else:
            self.held_reference.update(mode=0,speed=0.,movement_direction=[0.,0.,0.])
            if 'upper_body_velocity' in self.held_reference:
                self.held_reference['upper_body_velocity'] = [0.]*17

    def recovery_alignment(self, snapshot):
        ages=[snapshot.get('body_age_ms'),*snapshot.get('controller_age_ms',{}).values()]
        if len(ages)!=3 or any(age is None or age>100 for age in ages):
            return '等待连续新鲜数据'
        controls = snapshot['controllers']
        if any(c[k] for c in controls.values() for k in ('primary','secondary','click','menu')):
            return '请松开按键并回中摇杆'
        if any(abs(v)>=.15 for c in controls.values() for v in c['axes']):
            return '请松开按键并回中摇杆'
        if self.mode == 'pose':
            fields = snapshot['fields'];held = self.held_reference
            pose = R.from_rotvec(fields['smpl_pose'][-1])
            target = R.from_rotvec(held['smpl_pose'][-1])
            root = R.from_quat(fields['body_quat_w'][-1][[1,2,3,0]])
            target_root = R.from_quat(held['body_quat_w'][-1][[1,2,3,0]])
            if (np.max((pose.inv()*target).magnitude())>.35 or
                (root.inv()*target_root).magnitude()>.15 or
                np.max(np.linalg.norm(fields['smpl_joints'][-1]-held['smpl_joints'][-1],axis=-1))>.1):
                return '请回到断流前姿态'
        if self.mode in ('pose','vr3pt'):
            vr = self.calibration.apply(snapshot['raw_body'])
            held = self.held_reference
            position = held['vr_position'] if self.mode=='pose' else held['vr_3pt_position']
            orientation = held['vr_orientation'] if self.mode=='pose' else held['vr_3pt_orientation']
            target = R.from_quat(np.asarray(orientation).reshape(3,4)[:,[1,2,3,0]])
            rotation = R.from_quat(vr[:,3:][:,[1,2,3,0]])
            if (np.max(np.linalg.norm(vr[:,:3]-np.asarray(position).reshape(3,3),axis=-1))>.1 or
                np.max((rotation.inv()*target).magnitude())>.35):
                return '请回到断流前姿态'
        return ''

    def update_tracking(self, now, snapshot, allow_hold=True):
        self.recovery_checked_at = now
        # Receiver replacement and clock faults require a new explicit session.
        if (snapshot.get('receiver_id')!=self.receiver_id or snapshot.get('control_fault') or
            snapshot.get('body_clock_fault')):
            self.recovery_reason = 'PICO 接收器或时间戳已改变，需人工重新进入'
            return 'lost'
        ready = self.ready(snapshot)
        if not self.recovering:
            if ready:return 'live'
            if not allow_hold or self.cached_reference is None:return 'lost'
            self.begin_tracking_hold(now)
        if now-self.recovery_since>=self.recovery_seconds:
            self.recovery_reason = 'PICO 断流恢复超时，需人工重新进入'
            return 'lost'
        reason = self.recovery_alignment(snapshot) if ready else '等待新鲜身体和双手柄数据'
        if reason:
            self.recovery_fresh_since = None
            self.recovery_reason = reason
            return 'holding'
        if self.recovery_fresh_since is None:self.recovery_fresh_since = now
        self.recovery_reason = '等待连续新鲜数据'
        if now-self.recovery_fresh_since<self.recovery_stable_seconds:return 'holding'
        self.recovery_since = None
        self.recovery_fresh_since = None
        self.recovery_reason = ''
        self.held_reference = None
        self.last = None
        self.released = False
        self.previous = set()
        return 'resumed'

    def recovery_reference(self, now):
        if not self.recovering or now-self.recovery_since>=self.recovery_seconds:return None
        if self.mode!='pose':return deepcopy(self.held_reference)
        fields = deepcopy(self.held_reference)
        fields['frame_index'] = np.arange(self.index,self.index+5,dtype=np.int64)
        self.index += 1
        fields['timestamp_monotonic'] = np.array([now],dtype=np.float64)
        from .pose_wire import pack_pose_message
        return pack_pose_message(fields,version=3)

    def recovery_status(self):
        remaining = max(0.,self.recovery_seconds-(self.recovery_checked_at-self.recovery_since)) if self.recovering else 0.
        message = ('PICO 断流保持中，剩余 %.1f 秒：%s'%(remaining,self.recovery_reason)) if self.recovering else ''
        return dict(recovering=self.recovering,recovery_remaining_ms=round(remaining*1000),
                    recovery_window_ms=round(self.recovery_seconds*1000),recovery_message=message)
