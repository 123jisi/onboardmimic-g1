"""Read-only PN Link status and mapped-body preview; SDK remains in receiver."""
from pathlib import Path
import time
from onboardmimic.core.contracts import TrackingInput
from .state import read_state, fresh


class PNLinkInput(TrackingInput):
    id, label = 'pnlink', 'PN Link'

    def __init__(self):
        from onboardmimic.core.paths import STATE
        self.status_file = STATE/'pnlink/status.json'
        self._service_at = None
        self._service = {}

    def service_status(self):
        from .service import status
        now = time.monotonic()
        if self._service_at is None or now-self._service_at >= 1.:
            self._service = status()
            self._service_at = now
        return dict(self._service)

    def status(self):
        result = dict(connected=False, ready=False, receiver_running=False, reason='等待 PN Link 接收服务')
        try:
            data = read_state(self.status_file)
            age = time.monotonic()-data.get('updated_monotonic', 0)
            live = data.get('receiver_running') is True and 0 <= age <= .5
            received = data.get('received_monotonic')
            result.update(receiver_running=live, connected=live and data.get('connected') is True,
                          sdk_open=live and data.get('sdk_open') is True,
                          ready=live and fresh(data), device=data.get('device', {}),
                          capture_state=data.get('capture_state'), command=data.get('command', {}),
                          calibration=data.get('calibration', {}), endpoints=data.get('endpoints', {}),
                          body_age_ms=round((time.monotonic()-received)*1000, 1) if received is not None else None,
                          frame_id=data.get('frame_id'), receiver_id=data.get('receiver_id'),
                          reason=data.get('error', ''))
        except (OSError, ValueError, TypeError, KeyError):
            pass
        return result

    def skeleton(self):
        result = dict(available=False, joints=[], robot_output_enabled=False,
                      frame='SONIC SMPL local / meters / Z up', reason='等待 PN Link 身体数据')
        try:
            from onboardmimic.backends.sonic.pnlink.mapping import map_joints
            data = read_state(self.status_file)
            if not fresh(data):
                return result
            mapped = map_joints(data['joints'])
            result.update(available=True, joints=mapped['smpl_joints'].tolist(),
                          frame_id=data['frame_id'],
                          derived_joint_names=[n for n in ('LeftTiptoe','RightTiptoe') if n not in {r['name'] for r in data['joints']}],
                          reason='PN Link 身体参考预览；缺少的足尖使用参考实现的零局部旋转')
        except (OSError, ValueError, TypeError, KeyError):
            pass
        return result
