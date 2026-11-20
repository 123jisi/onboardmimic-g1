"""Optional bilateral hand authority, local calibration and observe-only preview.

Called under the gateway lock. No sockets, DDS or hardware SDK imports. The
single gateway transport publishes intent; the existing hand worker owns DDS.
"""
import copy
import json
import math
import logging
from pathlib import Path
import secrets
import time

import numpy as np
from scipy.spatial.transform import Rotation
from .hand_mapping import Mapping, FINGERS, CHANNELS, pose_array, relative_pose
from ....inputs.pnlink.state import atomic_write, fresh
from ..pico.hands import TOPIC


class HandCalibration:
    """Independent input, two-pose calibration and preview for one glove."""
    def __init__(self, reader, path, clock=time.monotonic, side='right'):
        if side not in ('left', 'right'):
            raise ValueError('未知手侧')
        self.side = side
        self.label = '左手' if side == 'left' else '右手'
        self.reader, self.path, self.clock = reader, Path(path), clock
        self.mode, self.closure = 'off', .25
        self.calibration = self.origin = None
        self.confirmed = self.key = None
        self.pose = self.at = self.frame = None
        self.capture = None
        self.capture_status = dict(stage='idle')
        self.session = ''
        self.blocked = False
        self.worker_id = None
        self.error = self.input_error = self.load_error = ''
        self.input_fault = ''
        self.last_fault = None
        self.last_worker = {}
        self.last_owned = None
        self.finish_task = None
        self.target = self.hold = None
        try:
            data = json.loads(self.path.read_text())
            if not isinstance(data, dict):
                raise ValueError('手部设置须为对象')
            if data.get('version') != 1:
                raise ValueError('未知手部设置版本')
            self.validate_settings(data['mode'], data['closure'])
            self.mode, self.closure = data['mode'], float(data['closure'])
            self.calibration, self.origin = data.get('calibration'), data.get('origin')
            if self.calibration is not None and (not isinstance(self.calibration, dict) or self.calibration.get('hand') != self.side):
                raise ValueError('标定手侧不匹配')
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, KeyError):
            self.mode, self.closure = 'off', .25
            self.calibration = self.origin = None
            self.load_error = '手部设置无法读取，请重新设置并标定；手控已关闭'

    @staticmethod
    def validate_settings(mode, closure):
        if (mode not in ('off', 'fingers', 'grasp') or type(closure) not in (int, float)
                or not math.isfinite(closure) or not .05 <= closure <= 1):
            raise ValueError('手模式须为 off/fingers/grasp，幅度须为 5%–100%')

    def save(self):
        atomic_write(self.path, dict(version=1, mode=self.mode, closure=self.closure,
                                     calibration=self.calibration, origin=self.origin,
                                     hands=getattr(self, 'selected', 'right')))

    def finishing(self):
        return bool(self.finish_task and self.finish_task['stage'] in ('sending', 'opening'))

    def fail(self, reason):
        if self.finishing():
            self.finish_task.update(stage='failed', error=reason)
        if not self.blocked:
            self.last_fault = dict(reason=reason, at_monotonic=self.clock(), session=self.session,
                input_age_ms=(self.clock()-self.at)*1000 if self.at is not None else None,
                input_error=self.input_error, worker=copy.deepcopy(self.last_worker))
            logging.getLogger('runtime_audit').info(json.dumps(dict(event='pnlink_hand_disabled', **self.last_fault), ensure_ascii=False))
        self.blocked = True
        self.error = reason

    def invalidate_calibration(self):
        self.confirmed = None
        self._cancel_capture('PN Link 校准已改变，请重新采样张手和握拳')
        if self.calibration is not None:
            self.calibration = self.origin = None
            self.save()
        self.blocked = True
        self.error = 'PN Link 校准已改变，请重新采样张手和握拳'

    def _cancel_capture(self, reason):
        if self.capture:
            self.capture_status = dict(stage='failed', error=reason)
            self.capture = None

    def update(self):
        raw = self.reader.snapshot().get('raw', {})
        key = (raw.get('receiver_id'), raw.get('device', {}), raw.get('avatar', {}),
               raw.get('calibration_revision', 0))
        if key != self.key:
            if self.key and key[0] == self.key[0] and key[3] != self.key[3]:
                self.invalidate_calibration()
            self.confirmed = None
            self.pose = self.at = self.frame = None
            self._cancel_capture('接收会话或设备已变化，请重新采样')
            if self.session:
                self.fail('接收会话或设备已变化，需退出并重新进入')
            self.key = copy.deepcopy(key)
        h = raw.get(self.side+'_hand', {})
        self.input_error = h.get('error') or ''
        valid = fresh(raw, self.clock()) and h.get('valid') is True
        if valid:
            try:
                frame, at = h['frame_id'], h['received_monotonic']
                if (type(frame) is not int or frame != raw.get('frame_id')
                        or type(at) not in (int, float) or not math.isfinite(at)
                        or not 0 <= self.clock()-at <= .25):
                    raise ValueError(self.label+'帧标识或时间无效')
                q = np.asarray(h['rotations_wxyz'], dtype=float)
                if (q.shape != (15, 4) or not np.isfinite(q).all()
                        or np.any((q*q).sum(axis=1) < .64) or np.any((q*q).sum(axis=1) > 1.44)):
                    raise ValueError(self.label+'旋转无效')
                if self.frame is not None and (frame < self.frame or at < self.at):
                    self.input_fault = self.label+'时间回退，需退出并重新进入'
                    raise ValueError(self.label+'时间回退')
                if self.frame != frame and self.at != at:
                    self.pose = pose_array(Rotation.from_quat(q[:, [1, 2, 3, 0]]).as_rotvec())
                    self.frame, self.at = frame, at
                    self._capture_frame()
            except (KeyError, TypeError, ValueError) as exc:
                valid = False
                self.input_error = str(exc)
        if not valid:
            self.pose = None
            self._cancel_capture(self.input_error or self.label+'数据缺失或过期')
        if self.capture and self.clock()-self.capture['started'] > 8:
            self._cancel_capture('手部采样超时，请保持姿势并重试')

    def input_ready(self):
        return self.pose is not None and self.at is not None and 0 <= self.clock()-self.at <= .25

    def mapping(self, mode=None):
        if self.calibration is None:
            raise ValueError('请采样'+self.label+'张手和握拳')
        return Mapping(self.calibration, mode or self.mode, self.closure, hand=self.side)

    def calibration_matches(self):
        return bool(self.origin and self.key and self.origin == dict(device=self.key[1], avatar=self.key[2]))

    def confirm(self, checked):
        if checked is not True or not self.input_ready() or not self.calibration_matches():
            raise ValueError('请检查当前穿戴和手指预览；设备变化须重新采样')
        self.mapping('grasp')
        self.confirmed = copy.deepcopy(self.key)

    def start_capture(self, kind, finger=None):
        if kind not in ('open', 'closed', 'finger') or kind == 'finger' and finger not in FINGERS:
            raise ValueError('未知手部采样动作')
        if self.capture or not self.input_ready():
            raise ValueError('请等待采样完成，并确认新鲜的'+self.label+'数据')
        if kind != 'open' and (not self.calibration or not self.calibration_matches()):
            raise ValueError('请先采样当前设备的张手姿态')
        if kind == 'finger' and self.calibration.get('version') != 2:
            raise ValueError('单指补采需要重新采样新版张手参考')
        self.capture = dict(kind=kind, finger=finger, started=self.clock(), poses=[], times=[])
        self.capture_status = dict(stage='running', kind=kind, finger=finger, frames=0, seconds=0)

    def _capture_frame(self):
        c = self.capture
        if c is None or self.at < c['started']:
            return
        c['poses'].append(self.pose.copy())
        c['times'].append(self.at)
        duration = c['times'][-1]-c['times'][0]
        self.capture_status.update(frames=len(c['poses']), seconds=round(duration, 1))
        if duration < 2:
            return
        self.capture = None
        try:
            if len(c['poses']) < 20:
                raise ValueError('新鲜手部帧不足，请重试')
            mean = np.stack([Rotation.from_rotvec(np.stack(c['poses'])[:, j]).mean().as_rotvec() for j in range(15)])
            jitter = max(float(np.max(np.linalg.norm(relative_pose(mean, p), axis=1))) for p in c['poses'])
            if jitter > .15:
                raise ValueError('采样时手指移动过大，请保持不动重试')
            if c['kind'] == 'open':
                calibration = dict(version=2, hand=self.side, open=mean.tolist())
            else:
                calibration = copy.deepcopy(self.calibration)
                if c['kind'] == 'closed':
                    calibration['closed'] = mean.tolist()
                    calibration.pop('finger_closed', None)
                    Mapping(calibration, 'grasp', hand=self.side)
                else:
                    calibration.setdefault('closed', calibration['open'])
                    calibration.setdefault('finger_closed', {})[c['finger']] = mean.tolist()
                    mapping = Mapping(calibration, 'fingers', allow_partial=True, hand=self.side)
                    from .hand_mapping import CHANNEL_FINGERS
                    if not mapping.valid[[i for i, f in enumerate(CHANNEL_FINGERS) if f == c['finger']]].all():
                        raise ValueError('该手指变化量不足，未保存')
            before = self.calibration, self.origin, self.confirmed
            self.calibration = calibration
            self.origin = copy.deepcopy(dict(device=self.key[1], avatar=self.key[2]))
            self.confirmed = copy.deepcopy(self.key) if 'closed' in calibration else None
            try:
                self.save()
            except Exception:
                self.calibration, self.origin, self.confirmed = before
                raise
            if self.confirmed is not None and self.error == 'PN Link 校准已改变，请重新采样张手和握拳':
                self.error = ''
            self.capture_status.update(stage='succeeded', jitter_rad=round(jitter, 4), error='')
        except (OSError, ValueError, TypeError) as exc:
            self.capture_status.update(stage='failed', error=str(exc))

    def preview(self):
        result = {}
        for mode in ('fingers', 'grasp'):
            try:
                m = self.mapping(mode)
                result[mode] = dict(ready=True, target=m.target(self.pose).tolist() if self.input_ready() else None,
                                    spans_rad=dict(zip(CHANNELS, m.spans.tolist())))
            except (ValueError, TypeError, KeyError) as exc:
                result[mode] = dict(ready=False, error=str(exc))
        return result


class PNLinkHands(HandCalibration):
    """One PN session and one packet for the selected, independently calibrated hands.

    Legacy attributes describe the right side so existing right-only callers and
    persisted calibration continue to work. New APIs expose both via ``sides``.
    """
    def __init__(self, reader, path, clock=time.monotonic):
        super().__init__(reader, path, clock)
        self.left = HandCalibration(reader, self.path.with_name(self.path.stem+'.left'+self.path.suffix), clock, 'left')
        self.sides = dict(left=self.left, right=self)
        self.selected = 'right'
        try:
            data = json.loads(self.path.read_text())
            if not isinstance(data, dict):
                raise ValueError('手部设置须为对象')
            saved = data.get('hands', 'right')
            self.selected_sides(saved)
            self.selected = saved
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, KeyError):
            self.mode = 'off'
            self.load_error = '手侧设置无效，手控已关闭'
        self.left.mode, self.left.closure = self.mode, self.closure

    @staticmethod
    def selected_sides(hand):
        if hand not in ('left', 'right', 'both'):
            raise ValueError('手侧须为 left/right/both')
        return ('left', 'right') if hand == 'both' else (hand,)

    def sampling(self):
        return any(h.capture is not None for h in self.sides.values())

    def update(self):
        super().update()
        self.left.update()

    def invalidate_calibration(self):
        super().invalidate_calibration()
        if hasattr(self, 'left'):
            self.left.invalidate_calibration()

    def capture_hands(self, hand, kind, finger=None):
        sides = self.selected_sides(hand)
        if self.sampling():
            raise ValueError('请等待手部采样完成')
        # A failed bilateral admission must not leave the other side sampling.
        try:
            for side in sides:
                self.sides[side].start_capture(kind, finger)
        except Exception:
            for side in sides:
                self.sides[side].capture = None
                self.sides[side].capture_status = dict(stage='failed', error='采样未开始，请确认所选手的数据和张手参考')
            raise

    def confirm_hands(self, hand, checked):
        sides = self.selected_sides(hand)
        previous = {s: self.sides[s].confirmed for s in sides}
        try:
            for side in sides:
                self.sides[side].confirm(checked)
        except Exception:
            for side in sides:
                self.sides[side].confirmed = previous[side]
            raise

    def configure(self, mode, closure, owned=False, hand=None):
        selected = self.selected if hand is None else hand
        self.selected_sides(selected)
        if selected != self.selected and (owned or self.sampling() or self.finishing()):
            raise ValueError('请先退出遥操并等待手部操作完成再修改手侧')
        self.validate_settings(mode, closure)
        if self.finishing() and (mode != 'off' or closure != self.closure):
            raise ValueError('请等待手部结束清零完成')
        if self.sampling() and (mode != 'off' or closure != self.closure):
            raise ValueError('请等待手部采样完成')
        if owned and (mode != 'off' or closure != self.closure):
            raise ValueError('请先退出 PN Link 遥操再修改手模式或幅度')
        before = self.mode, self.closure, self.selected
        self.mode, self.closure, self.selected = mode, float(closure), selected
        try:
            self.save()
        except Exception:
            self.mode, self.closure, self.selected = before
            raise
        self.left.mode, self.left.closure = self.mode, self.closure
        self.load_error = ''
        if mode == 'off':
            self.revoke(open_hand=True)

    def revoke(self, reason='', open_hand=False):
        if not open_hand and self.finishing():
            self.finish_task.update(stage='failed', error=reason or '控制状态改变，已停止清零')
            self.last_owned = None
        if open_hand and self.last_owned and not self.finishing():
            self.finish_task = dict(self.last_owned, stage='sending', error='',
                                    started=self.clock(), deadline=self.clock()+8.)
        self.session = ''
        self.worker_id = None
        self.target = self.hold = None
        if not self.blocked:
            self.error = reason

        self.left.target = self.left.hold = None

    def enter(self):
        if self.finishing():
            raise ValueError('请等待手部结束清零完成')
        self.last_owned = None
        self.finish_task = None
        self.session = secrets.token_hex(16)
        self.blocked = False
        self.worker_id = None
        self.error = ''
        self.target = self.hold = None
        self.left.blocked = False
        self.left.error = ''
        self.input_fault = self.left.input_fault = ''
        self.left.target = self.left.hold = None

    def side_status(self, hand):
        h = self.sides[hand]
        return dict(input_ready=h.input_ready(), input_age_ms=round((self.clock()-h.at)*1000, 2) if h.at is not None else None,
                    calibration_confirmed=h.confirmed is not None and h.confirmed == h.key,
                    calibration_present=h.calibration is not None, capture=dict(h.capture_status),
                    preview=h.preview(), target=h.target, error=h.error or h.load_error or h.input_fault or h.input_error)

    def status(self, link=None):
        worker = link.status() if link else dict(fresh=False, error='手部适配器未配置')
        legacy = self.side_status('right')
        legacy['error'] = self.error or self.load_error
        return dict(legacy, hand=self.selected, sides={s:self.side_status(s) for s in self.sides},
                    mode=self.mode, closure=self.closure, sampling=self.sampling(),
                    active=bool(self.session and not self.blocked and self.mode!='off'), worker=worker,
                    finish=({k:v for k,v in self.finish_task.items() if k not in ('deadline','started')} if self.finish_task else None),
                    last_fault=copy.deepcopy(self.last_fault))

    def finish_message(self, link):
        task, status = self.finish_task, link.status()
        sides = task.get('sides', ['right'])
        error = ''
        if self.clock() > task['deadline']:
            error = '手部结束清零超时'
        elif not status.get('fresh') or status.get('bridge_id') != task['bridge_id']:
            error = '清零时手部适配器失效或重启'
        elif status.get('finish_open_supported') is not True or status.get('observe_only') is not False:
            error = '手部适配器不支持主动结束清零'
        elif status.get('last_session') not in (None, '', task['session']):
            error = '手部已由其他会话接管，取消旧会话清零'
        reports = {s:status.get('sides', {}).get(s, {}) for s in sides}
        progress = {}
        for side, fb in reports.items():
            label = self.sides[side].label
            if not fb.get('ready'):
                error = error or '清零时'+label+'反馈失效'
            if fb.get('finish_session') == task['session']:
                progress[side] = fb.get('finish_stage')
                if fb.get('finish_stage') == 'failed':
                    error = error or label+'：'+(fb.get('finish_error') or '清零失败')
        task['side_stages'] = progress
        if error:
            task.update(stage='failed', error=error)
            self.last_owned = None
            return None
        if all(progress.get(s) == 'succeeded' for s in sides):
            task.update(stage='succeeded', error='')
            self.last_owned = None
            return None
        if any(stage == 'opening' for stage in progress.values()):
            task['stage'] = 'opening'
        link.sequence += 1
        packet = dict(protocol=2, source='pnlink', bridge_id=task['bridge_id'], session=task['session'],
                      sequence=link.sequence, at_ns=link.clock_ns(), active=False, left=None, right=None,
                      finish_open=True, finish_sides=sides)
        return TOPIC+json.dumps(packet, separators=(',', ':'), allow_nan=False).encode()

    def message(self, gateway, link):
        if self.finishing():
            return self.finish_message(link)
        status = link.status()
        selected = self.selected_sides(self.selected)
        fb = status.get('sides', {}).get('right', {})
        self.last_worker = dict(fresh=status.get('fresh'), age_ms=status.get('age_ms'),
            feedback_ready=fb.get('ready'), feedback_age_ms=fb.get('age_ms'), feedback_samples=fb.get('samples'),
            sides=copy.deepcopy(status.get('sides', {})), active=status.get('active'), error=status.get('error') or fb.get('error'))
        active = bool(self.mode != 'off' and self.session and not self.blocked and gateway.phase == 'active'
                      and gateway.pnlink_owned() and not gateway.mode_pending and not gateway.task_active())
        goals = dict(left=None, right=None)
        if active:
            try:
                if not status.get('fresh') or status.get('observe_only') is not False or 2 not in status.get('intent_protocols', []) or status.get('finish_open_supported') is not True:
                    raise ValueError('连续手部适配器未就绪，需重新进入')
                if 'left' in selected and set(status.get('continuous_sides', [])) != {'left', 'right'}:
                    raise ValueError('手部适配器尚未支持双手连续目标')
                if self.worker_id is not None and self.worker_id != status['bridge_id']:
                    raise ValueError('手部适配器已重启，需重新进入')
                self.worker_id = status['bridge_id']
                if status.get('last_session') == self.session and status.get('error'):
                    raise ValueError(status['error'])
                # Validate every selected side before sending either target.
                for side in selected:
                    h = self.sides[side]
                    if h.input_fault or not h.input_ready() or side == 'left' and h.blocked:
                        raise ValueError(h.input_fault or h.input_error or h.label+'数据缺失或过期，需重新进入')
                    if h.confirmed is None or h.confirmed != h.key:
                        raise ValueError('请检查并确认'+h.label+'标定后重新进入')
                    feedback = status.get('sides', {}).get(side, {})
                    if not feedback.get('ready'):
                        raise ValueError(h.label+'反馈缺失或过期，需重新进入')
                    if status.get('session') == self.session and feedback.get('error'):
                        raise ValueError(h.label+'：'+feedback['error'])
                    mapped = h.mapping().target(h.pose).tolist()
                    if gateway.pnlink_session.paused:
                        if h.hold is None:
                            h.hold = h.target if h.target is not None else mapped
                        goals[side] = h.hold
                    else:
                        h.hold = None
                        goals[side] = mapped
                for side in selected:
                    self.sides[side].target = goals[side]
                self.last_owned = dict(bridge_id=self.worker_id, session=self.session, sides=list(selected))
            except Exception as exc:
                self.fail(str(exc))
                active = False
        if not status.get('fresh'):
            return None
        link.sequence += 1
        data = dict(protocol=2, source='pnlink', bridge_id=status['bridge_id'], session=self.session if active else '',
                    sequence=link.sequence, at_ns=link.clock_ns(), active=active,
                    left=goals['left'] if active else None, right=goals['right'] if active else None)
        return TOPIC+json.dumps(data, separators=(',', ':'), allow_nan=False).encode()
