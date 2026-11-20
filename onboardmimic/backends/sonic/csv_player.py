"""Local 50 Hz joint-reference player; no sockets, Torch, or hardware imports.

The wire index always advances, including during pause and mode admission. Only
the file clock freezes. A real 64-frame future window keeps the native 10xstep5 encoder supplied.
"""
import argparse
import hashlib
import io
import json
import math
import time
from pathlib import Path

import numpy as np

from .wire import pack_pose_message

# Exact native policy_parameters.hpp::mujoco_to_isaaclab gather. Names in
# upstream comments are inconsistent: use the executable mapping, not labels.
# reference[i] == measured_hardware[REFERENCE_TO_HARDWARE[i]].
REFERENCE_TO_HARDWARE = (0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10,
                         16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28)


class CSVClip:
    mode = "joint_playback"
    def __init__(self, path, *, fps, joint_order, root_body_index=0):
        if type(fps) not in (int, float) or not math.isfinite(fps) or fps != 50:
            raise ValueError('阶段 1 仅接受显式 50 Hz CSV；其他帧率须先重采样')
        if joint_order != 'sonic_reference':
            raise ValueError('必须声明 joint_order=sonic_reference（原生 MotionSequence 顺序）')
        if type(root_body_index) is not int or root_body_index < 0:
            raise ValueError('root_body_index 无效')
        self.path = Path(path).resolve()
        self.fps, self.hashes = float(fps), {}
        arrays = []
        for name in ('joint_pos.csv', 'joint_vel.csv', 'body_quat.csv'):
            source = (self.path / name).resolve()
            if source.parent != self.path or not source.is_file():
                raise ValueError('CSV 缺失或越过动作目录：'+name)
            if source.stat().st_size > 64 * 1024 * 1024:
                raise ValueError('CSV 超过 64 MiB：'+name)
            raw = source.read_bytes()
            self.hashes[name] = hashlib.sha256(raw).hexdigest()
            try:
                header = raw.split(b'\n', 1)[0].decode('utf-8-sig').strip().split(',')
                if not header or any(not cell.strip() for cell in header):
                    raise ValueError('缺少表头')
                try:
                    [float(cell) for cell in header]
                except ValueError:
                    pass
                else:
                    raise ValueError('首行是数值；缺少 CSV 表头')
                value = np.loadtxt(io.BytesIO(raw), delimiter=',', skiprows=1,
                                   dtype=np.float32, ndmin=2)
                if len(header) != value.shape[1]:
                    raise ValueError('表头与数据列数不一致')
            except (ValueError, OSError) as exc:
                raise ValueError('CSV 内容无效：'+name) from exc
            if not 2 <= len(value) <= 30000 or not np.isfinite(value).all():
                raise ValueError('CSV 帧数或有限数值检查失败：'+name)
            arrays.append(value)
        q, dq, quat = arrays
        if q.shape[1] != 29 or dq.shape != q.shape or len(quat) != len(q):
            raise ValueError('CSV 需要一致帧数及 29 关节')
        if quat.shape[1] % 4 or root_body_index >= quat.shape[1] // 4:
            raise ValueError('body_quat 必须为 wxyz 四元数，root_body_index 越界')
        quat = quat[:, root_body_index*4:root_body_index*4+4].copy()
        norms = np.linalg.norm(quat, axis=1)
        if np.any(np.abs(norms - 1.) > .02):
            raise ValueError('根四元数不是单位 wxyz 四元数')
        quat /= norms[:, None]
        for value in (q, dq, quat):
            value.flags.writeable = False
        self.q, self.dq, self.quat = q, dq, quat
        self.root_body_index = root_body_index
        self.frames = len(q)
        # Each sample occupies one interval; the last is held for 20 ms.
        self.duration = self.frames / self.fps

    def metadata(self):
        return dict(name=self.path.name, frames=self.frames, fps=self.fps,
                    duration_seconds=self.duration, joint_order='sonic_reference',
                    quaternion_order='wxyz', root_body_index=self.root_body_index,
                    hashes=dict(self.hashes))

    def alignment_error(self, hardware_q):
        q = np.asarray(hardware_q)[list(REFERENCE_TO_HARDWARE)]
        return float(np.max(np.abs(q - self.q[0])))

    def tilt_error(self, hardware_quat):
        def gravity(q):
            w, x, y, z = q / np.linalg.norm(q)
            return np.array([2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)])
        # Heading is reinitialized by native admission. Compare tilt, invariant
        # to world yaw, so an incompatible first root pose cannot jump in.
        dot = np.dot(gravity(np.asarray(hardware_quat)), gravity(self.quat[0]))
        return float(np.arccos(np.clip(dot, -1., 1.)))


class CSVPlayer:
    producer_timeout = .15
    # Native mode0 requests 0,5,...,45. One extra frame permits the
    # post-control cursor advance before the next packet is merged.
    # 18 spare samples tolerate receive-side conflate/jitter without stalling
    # the file cursor; they add no delay to the current reference.
    lookahead_frames = 64

    def __init__(self):
        self.clip = None
        self.state = 'unloaded'
        self.error = ''
        self.position = 0.
        self.stream_index = 0
        self.confirmed = False
        self.produced_at = None
        self.clock_at = None
        self.started_at = None

    def load(self, clip):
        self.clip = clip
        self.state, self.error = 'loaded', ''
        self.position = 0.
        self.clear_stream()

    def clear_stream(self):
        self.stream_index = 0
        self.confirmed = False
        self.produced_at = self.clock_at = self.started_at = None

    def start(self, now):
        if self.clip is None:
            raise ValueError('请先加载 CSV 动作')
        self.clear_stream()
        self.state, self.error, self.position = 'playing', '', 0.
        self.started_at = self.clock_at = now

    def confirm(self, now):
        self.confirmed = True
        self.clock_at = now

    def pause(self):
        if self.state != 'playing' or not self.confirmed:
            raise ValueError('动作尚未开始播放或模式未确认')
        self.state = 'paused'

    def resume(self, now):
        if self.state != 'paused':
            raise ValueError('动作未暂停')
        self.state, self.clock_at = 'playing', now

    def end(self, state='loaded', error=''):
        self.state = state if self.clip is not None else 'unloaded'
        self.error = error
        self.clear_stream()

    def healthy(self, now):
        stamp = self.produced_at if self.produced_at is not None else self.started_at
        return stamp is not None and 0 <= now - stamp <= self.producer_timeout

    def packet(self, now):
        if self.state not in ('playing', 'paused'):
            return None
        if not self.healthy(now):
            self.end('error', '本地播放器产出超时')
            return None
        if self.state == 'playing' and self.confirmed:
            self.position += max(0., now - self.clock_at)
        self.clock_at = now
        if self.position >= self.clip.duration - 1e-9:
            self.position = self.clip.duration
            self.end('finished')
            return None
        frame = min(int(self.position * self.clip.fps + 1e-7), self.clip.frames - 1)
        hold = self.state == 'paused' or not self.confirmed
        offsets = np.arange(self.lookahead_frames, dtype=np.int64)
        samples = np.minimum(frame + (0 if hold else offsets), self.clip.frames - 1)
        if np.ndim(samples) == 0:
            samples = np.full(self.lookahead_frames, frame, dtype=np.int64)
        fields = dict(
            joint_pos=self.clip.q[samples],
            joint_vel=np.zeros((self.lookahead_frames, 29), dtype=np.float32) if hold else self.clip.dq[samples],
            body_quat=self.clip.quat[samples, None, :],
            frame_index=self.stream_index + offsets,
            timestamp_monotonic=np.array([now], dtype=np.float64))
        version = 1
        if self.clip.mode == "smpl_playback":
            fields.pop("body_quat")
            fields.update({k:v[samples] for k,v in self.clip.arrays.items()})
            fields["timestamp_realtime"] = np.array([time.time()], dtype=np.float64)
            if hold:
                fields["joint_vel"] = np.zeros((self.lookahead_frames,29), dtype=np.float32)
            version = 3
        packet = pack_pose_message(fields, version=version)
        self.stream_index += 1
        self.produced_at = now
        return packet

    def snapshot(self):
        return dict(state=self.state, position_seconds=self.position,
                    error=self.error, mode_confirmed=self.confirmed,
                    reference_window_frames=self.lookahead_frames,
                    clip=self.clip.metadata() if self.clip else None)


def main():
    parser = argparse.ArgumentParser(description='读取并验证 CSV，不启动控制器、不发送数据')
    parser.add_argument('path', type=Path)
    parser.add_argument('--fps', type=float, required=True)
    parser.add_argument('--joint-order', choices=['sonic_reference'], required=True)
    parser.add_argument('--root-body-index', type=int, default=0)
    args = parser.parse_args()
    clip = CSVClip(args.path, fps=args.fps, joint_order=args.joint_order,
                   root_body_index=args.root_body_index)
    print(json.dumps(clip.metadata(), indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
