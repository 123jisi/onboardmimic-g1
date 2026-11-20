"""Single-owner SDK receiver. No SONIC, ZMQ, DDS or motor-control imports."""
import argparse
import fcntl
import importlib.util
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import resource
import signal
import socket
import time

from .state import atomic_write
from .model import BODY_NAMES, HAND_NAMES
from .autocapture import AutoCapture


class Receiver:
    def __init__(self, sdk, local_ip, server_ip, local_port=8002, server_port=8080,
                 clock=time.monotonic, auto_capture=False):
        self.sdk, self.clock = sdk, clock
        self.id = secrets.token_hex(16)
        self.started = self.clock()
        self.last_event = None
        self.source_time = None
        self.frame = 0
        self.pending = None
        self.handle = None
        self.owned_command = None
        self.requests = {}
        self.auto_capture = AutoCapture(clock) if auto_capture else None
        self.data = dict(protocol=1, data_source='sdk', receiver_id=self.id,
                         receiver_running=False, sdk_open=False, connected=False,
                         capture_state='unknown', device={}, joints=[], frame_id=0, calibration_revision=0,
                         received_monotonic=None, source_timestamp=None,
                         command={}, calibration=dict(query_supported=False, evidence='unknown'),
                         endpoints=dict(local_ip=local_ip, local_port=local_port,
                                        server_ip=server_ip, server_port=server_port), error='')
        self.app = sdk.MCPApplication()
        settings = sdk.MCPSettings()
        settings.set_bvh_data(sdk.MCPBvhData.Binary)
        settings.set_bvh_transformation(sdk.MCPBvhDisplacement.Enable)
        settings.set_bvh_rotation(sdk.MCPBvhRotation.YXZ)  # Reference executable, not its YZX comment.
        settings.SetSettingsUDPEx(local_ip, local_port)
        settings.SetSettingsUDPServer(server_ip, server_port)
        self.app.set_settings(settings)

    def open(self):
        ok, detail = self.app.open()
        if not ok:
            raise RuntimeError('PN Link SDK open failed: ' + str(detail))
        self.data.update(sdk_open=True, receiver_running=True)
        self.data['sdk_open_succeeded'] = True

    def close(self):
        if self.app.is_opened():
            ok, detail = self.app.close()
            self.data['sdk_closed_cleanly'] = ok
            if not ok:
                self.data['error'] = 'SDK close failed: ' + str(detail)
        self.data.update(receiver_running=False, sdk_open=False, connected=False)
        if self.pending:
            self.data['command']['stage'] = 'unknown'
            self.data['command']['error'] = '接收器已关闭；设备命令结果未知'

    def submit(self, request, automatic=False):
        action = request.get('action')
        if request.get('receiver_id') != self.id:
            raise ValueError('PN Link 接收会话已变化，请刷新')
        key = request.get('request_id')
        if not isinstance(key, str) or not 1 <= len(key) <= 100:
            raise ValueError('无效 PN Link 命令标识')
        if key in self.requests:
            if self.requests[key] != action:
                raise ValueError('命令标识已用于另一操作')
            return dict(accepted=True, request_id=key)
        types = dict(start_capture=self.sdk.EMCPCommand.CommandStartCapture,
                     stop_capture=self.sdk.EMCPCommand.CommandStopCapture,
                     calibrate=self.sdk.EMCPCommand.CommandCalibrateMotion)
        if action not in types:
            raise ValueError('未知 PN Link 命令；SDK 不支持取消校准')
        if not self.data['sdk_open']:
            raise ValueError('PN Link SDK 未打开')
        if action == 'calibrate' and not self.data['connected']:
            raise ValueError('PN Link 设备尚未应答')
        if self.pending:
            raise ValueError('已有在途设备命令，等待明确结果')
        if action == 'calibrate' and self.data['capture_state'] != 'capturing':
            raise ValueError('请先开始采集并等待有效人体帧')
        # Retain the exact command handle: unrelated/late replies cannot finish this request.
        cmd = self.sdk.MCPCommand()
        cmd.create_command(types[action])
        error = self.app.api.contents.QueuedServerCommand(cmd.handle, self.app._handle)
        if error != self.sdk.MCPError.NoError:
            cmd.destroy_command(cmd.handle)
            raise RuntimeError('PN Link SDK queue failed: ' + str(error))
        self.owned_command = cmd
        self.handle = int(cmd.handle.value)
        self.pending = action
        self.requests[key] = action
        if len(self.requests) > 128:
            self.requests.pop(next(iter(self.requests)))
        self.data['command'] = dict(action=action, request_id=key, stage='running',
                                    started_monotonic=self.clock(), progress=None, error='',
                                    automatic=automatic)
        if self.auto_capture and not automatic:
            self.auto_capture.manual(action)
        if action == 'calibrate':
            self.data['calibration_revision'] += 1
            self.data['calibration'] = dict(query_supported=False, evidence='unknown')
        return dict(accepted=True, request_id=key)

    def avatar(self, event):
        stamp = float(event.timestamp)
        if not math.isfinite(stamp):
            raise ValueError('PN Link SDK 时间戳无效')
        if self.source_time is not None and stamp < self.source_time:
            raise RuntimeError('PN Link SDK 时间戳回退，重建接收会话后人工进入')
        if stamp > 0 and stamp == self.source_time:
            return
        avatar = self.sdk.MCPAvatar(event.event_data.avatar_handle)
        rows, hands, hand_errors = [], {side: {} for side in HAND_NAMES}, {}
        for joint in avatar.get_joints():
            name = joint.get_name()
            side = next((s for s, names in HAND_NAMES.items() if name in names), None)
            if side is not None:
                hand = hands[side]
                try:
                    q = list(joint.get_local_rotation())
                    if (name in hand or len(q) != 4 or not all(math.isfinite(v) for v in q)
                            or not .64 <= sum(v*v for v in q) <= 1.44):
                        raise ValueError('手指旋转无效或重复')
                    hand[name] = q
                except Exception:
                    hand_errors[side] = '手指关节无效：' + name
                continue
            if name not in BODY_NAMES:
                continue  # Optional fingers cannot gate the 29-DoF body.
            q, p = joint.get_local_rotation(), joint.get_local_position()
            if q is None or p is None:
                raise ValueError('PN Link 关节数据缺失')
            q, p = list(q), list(p)
            if (len(q) != 4 or len(p) != 3 or not all(math.isfinite(v) for v in q+p)
                    or not .64 <= sum(v*v for v in q) <= 1.44):
                raise ValueError('PN Link 关节数值无效')
            rows.append(dict(name=name, rotation_wxyz=q, position_xyz=p))
        if not rows or len(rows) > 128:
            raise ValueError('PN Link 人体关节数无效')
        self.frame += 1
        self.source_time = stamp
        received = self.clock()
        for side, names in HAND_NAMES.items():
            hand, error = hands[side], hand_errors.get(side, '')
            valid = len(hand) == 15 and not error
            self.data[side+'_hand'] = dict(valid=valid, frame_id=self.frame,
                received_monotonic=received, rotations_wxyz=[hand[n] for n in names] if valid else [],
                error=error or ('' if valid else '缺少'+('左' if side == 'left' else '右')+'手指关节'))
        self.data.update(joints=rows, frame_id=self.frame,
                         avatar=dict(index=avatar.get_index(), name=avatar.get_name()),
                         source_timestamp=stamp, received_monotonic=received)
        if (not self.pending and self.data['command'].get('stage') != 'unknown'
                and self.data['capture_state'] != 'stopped'):
            self.data['capture_state'] = 'capturing'

    def reply(self, reply):
        if not self.pending or int(reply._commandHandle) != self.handle:
            return
        cmd, s = self.owned_command, self.data['command']
        if reply._replay == self.sdk.MCPReplay.MCPReplay_Running and self.pending == 'calibrate':
            progress = self.sdk.MCPCalibrateMotionProgress(cmd.get_progress(reply._commandHandle))
            step, name = progress.get_step_current_pose()
            value = None
            steps = self.sdk.MCPCalibrateMotionProgressStep
            if step == steps.CalibrateMotionProgressStep_Countdown:
                value, name = progress.get_countdown_current_pose()
            elif step == steps.CalibrateMotionProgressStep_Progress:
                value, name = progress.get_progress_current_pose()
            s.update(pose=name, step=step, progress=value)
        elif reply._replay == self.sdk.MCPReplay.MCPReplay_Result:
            code = cmd.get_result_code(reply._commandHandle)
            s.update(stage='succeeded' if code == 0 else 'failed', result_code=code,
                     error=cmd.get_result_message(reply._commandHandle) if code else '')
            if self.pending == 'calibrate' and code == 0:
                self.data['calibration'] = dict(query_supported=False, evidence='sdk_command_succeeded',
                                                completed_at_unix=time.time())
            elif self.pending == 'stop_capture' and code == 0:
                self.data.update(capture_state='stopped', received_monotonic=None)
            elif self.pending == 'start_capture' and code == 0:
                self.data['capture_state'] = 'starting'
            cmd.destroy_command(reply._commandHandle)
            self.pending = self.handle = self.owned_command = None

    def poll(self):
        for event in self.app.poll_next_event():
            if event.event_type == self.sdk.MCPEventType.Error:
                raise RuntimeError('PN Link SDK error event: ' + str(event.event_data.error))
            self.last_event = self.clock()
            if event.event_type == self.sdk.MCPEventType.AvatarUpdated:
                self.avatar(event)
            elif event.event_type == self.sdk.MCPEventType.Notify:
                n = event.event_data.notifyData
                if n._notify == self.sdk.MCPEventNotify.Notify_SystemUpdated:
                    device = self.sdk.MCPSystem(n._notifyHandle)
                    self.data['device'] = dict(serial=device.get_master_serial_number(),
                                               version=device.get_master_version())
            elif event.event_type == self.sdk.MCPEventType.CommandReply:
                self.reply(event.event_data.commandRespond)
        self.data['connected'] = self.last_event is not None and 0 <= self.clock()-self.last_event <= 2
        self.data['metrics'] = dict(uptime_seconds=self.clock()-self.started,
                                    cpu_seconds=time.process_time(),
                                    peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
        if self.pending and self.clock()-self.data['command']['started_monotonic'] > 90:
            self.data['command'].update(stage='unknown', error='设备命令超时，结果未知；等待最终应答')
        if self.auto_capture:
            self.auto_capture.tick(self)
        return self.data


def load_sdk(root):
    root = Path(root).resolve()
    manifest = json.loads((root/'sdk-install.json').read_text())
    for name, expected in manifest['installed_sha256'].items():
        path = (root/name).resolve(strict=True)
        if root not in path.parents or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('PN Link SDK installation hash mismatch: ' + name)
    spec = importlib.util.spec_from_file_location('onboard_pnlink_sdk', root/'mocap_api.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sdk-root', type=Path, required=True)
    p.add_argument('--local-ip', required=True)
    p.add_argument('--server-ip', required=True)
    p.add_argument('--local-port', type=int, default=8002)
    p.add_argument('--server-port', type=int, default=8080)
    p.add_argument('--status', type=Path, required=True)
    p.add_argument('--socket', type=Path, required=True)
    p.add_argument('--duration', type=float, default=0, help='0: service; positive: bounded no-command probe')
    p.add_argument('--auto-capture', action='store_true', help='Start device capture when receiver is explicitly started; never calibrates or starts the robot')
    a = p.parse_args()
    if a.auto_capture and a.duration:
        p.error('Bounded no-command probes cannot enable automatic capture')
    runtime = Path('/run/user')/str(os.getuid())
    if not runtime.is_dir():
        p.error('PN Link single-owner lock requires the user runtime directory /run/user/UID')
    global_owner = (runtime/'onboardmimic-pnlink-sdk.lock').open('a')
    fcntl.flock(global_owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
    a.status.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    owner = a.status.with_suffix('.lock').open('a')
    fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
    a.socket.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    # Socket and lock always share this receiver's private directory.
    if a.socket.parent.resolve() != a.status.parent.resolve():
        p.error('socket and status must share the receiver state directory')
    if a.socket.exists():
        a.socket.unlink()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(a.socket)); os.chmod(a.socket, 0o600)
    server.listen(4); server.setblocking(False)
    receiver = None
    atomic_write(a.status, dict(protocol=1, data_source='sdk', receiver_running=False,
                                error='SDK initializing', updated_monotonic=time.monotonic()))
    stopped = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
    start = time.monotonic(); last_write = 0
    try:
        receiver = Receiver(load_sdk(a.sdk_root), a.local_ip, a.server_ip, a.local_port, a.server_port,
                            auto_capture=a.auto_capture)
        receiver.open()
        while not stopped[0] and (not a.duration or time.monotonic()-start < a.duration):
            receiver.poll()
            try:
                conn, _ = server.accept()
            except BlockingIOError:
                conn = None
            if conn:
                with conn:
                    conn.settimeout(.05)
                    try:
                        raw = bytearray()
                        while b'\n' not in raw and len(raw) <= 4096:
                            block = conn.recv(4097-len(raw))
                            if not block:
                                break
                            raw.extend(block)
                        if len(raw) > 4096 or not raw.endswith(b'\n'):
                            raise ValueError('无效 PN Link 请求')
                        if a.duration:
                            raise ValueError('只收数探针不接受设备命令')
                        result = receiver.submit(json.loads(raw))
                    except (ValueError, RuntimeError, OSError) as exc:
                        result = dict(error=str(exc))
                    try:
                        conn.sendall(json.dumps(result).encode()+b'\n')
                    except OSError:
                        pass
            now = time.monotonic()
            if now-last_write >= .02:
                atomic_write(a.status, dict(receiver.data, updated_monotonic=now, updated_at_unix=time.time()))
                last_write = now
            time.sleep(.005)
    except Exception as exc:
        if receiver is not None:
            receiver.data['error'] = type(exc).__name__ + ': ' + str(exc)
        else:
            atomic_write(a.status, dict(protocol=1, data_source='sdk', receiver_running=False,
                                        error=type(exc).__name__ + ': ' + str(exc), updated_monotonic=time.monotonic()))
        raise
    finally:
        if receiver is not None:
            try:
                receiver.close()
            finally:
                atomic_write(a.status, dict(receiver.data, receiver_running=False, updated_monotonic=time.monotonic(), updated_at_unix=time.time()))
        server.close(); a.socket.unlink(missing_ok=True); owner.close(); global_owner.close()


if __name__ == '__main__':
    main()
