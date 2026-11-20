"""Explicit PN Link authority, calibration evidence, pause and bounded resume."""
import copy
import numpy as np
from scipy.spatial.transform import Rotation as R
from ..wire import pack_pose_message


class PNLinkSession:
    def __init__(self, reader):
        self.reader = reader
        self.active = self.paused = False
        self.receiver_id = self.confirmed_receiver = None
        self.confirmed_device = None
        self.confirmed_avatar = None
        self.error = ''
        self.hold = self.last_fields = None
        self.index = 0

    def evidence_key(self):
        s = self.reader.snapshot()
        raw = s.get('raw', {})
        return s.get('receiver_id'), raw.get('device', {}), raw.get('avatar', {})

    def ready(self):
        return bool(self.reader.snapshot()['ready'])

    def confirm(self):
        if not self.ready():
            raise ValueError('请先开始采集并检查新鲜的人体预览')
        self.confirmed_receiver, self.confirmed_device, self.confirmed_avatar = copy.deepcopy(self.evidence_key())

    def calibrated(self):
        return (self.confirmed_receiver is not None and self.evidence_key() ==
                (self.confirmed_receiver, self.confirmed_device, self.confirmed_avatar))

    def clear_calibration(self):
        self.confirmed_receiver = self.confirmed_device = self.confirmed_avatar = None

    def enter(self):
        if not self.ready() or not self.calibrated():
            raise ValueError('请预览并确认当前穿戴校准，再人工进入')
        self.active = True
        self.paused = False
        self.receiver_id = self.reader.snapshot()['receiver_id']
        self.hold = self.last_fields = None
        self.index = 0
        self.error = ''

    def revoke(self, reason=''):
        self.active = self.paused = False
        self.hold = self.last_fields = None
        self.error = reason

    def pause(self):
        if not self.active or not self.ready() or self.last_fields is None:
            raise ValueError('PN Link 尚未发布有效参考，无法暂停')
        self.hold = {k: v.copy() for k, v in self.last_fields.items()}
        # Freeze one pose, not the historical five-frame motion. Sliding an
        # unchanged moving window would keep nonzero motion in the encoder.
        for key in ('smpl_pose','smpl_joints','body_quat_w','joint_pos','joint_vel'):
            self.hold[key] = np.repeat(self.last_fields[key][-1:], 5, axis=0)
        self.hold['joint_vel'].fill(0)
        self.paused = True

    def resume(self):
        if not self.active or not self.paused or not self.ready():
            raise ValueError('PN Link 未暂停或数据已过期')
        fields = self.reader.snapshot()['fields']
        a, b = R.from_rotvec(fields['smpl_pose'][-1]), R.from_rotvec(self.hold['smpl_pose'][-1])
        qa = R.from_quat(fields['body_quat_w'][-1][[1, 2, 3, 0]])
        qb = R.from_quat(self.hold['body_quat_w'][-1][[1, 2, 3, 0]])
        if np.max((a.inv()*b).magnitude()) > .35 or (qa.inv()*qb).magnitude() > .15:
            raise ValueError('恢复姿态偏差过大，请回到冻结姿态后再恢复')
        self.paused = False
        self.hold = None

    def reference(self, now):
        if not self.active or not self.ready() or not self.calibrated():
            return None
        fields = {k: v.copy() for k, v in (self.hold if self.paused else self.reader.snapshot()['fields']).items()}
        fields['frame_index'] = np.arange(self.index, self.index+5, dtype=np.int64)
        fields['timestamp_monotonic'] = np.array([now], dtype=np.float64)
        self.index += 1  # Sliding five-frame window, including while paused.
        self.last_fields = fields
        return pack_pose_message(fields, version=3)

    def status(self):
        s = self.reader.snapshot()
        return dict(active=self.active, paused=self.paused, ready=self.ready(),
                    calibration_confirmed=self.calibrated(), calibration_evidence='operator_confirmed' if self.calibrated() else 'unknown',
                    sdk_calibration=s.get('raw', {}).get('calibration', {}),
                    command=s.get('raw', {}).get('command', {}), error=self.error,
                    receiver_id=s.get('receiver_id'), mapping_ms=s.get('mapping_ms'))
