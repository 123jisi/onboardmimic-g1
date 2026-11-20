"""Read-only PN snapshots -> bounded 50 Hz SMPL window. Never publishes."""
from collections import deque
import math
from pathlib import Path
import threading
import time
import numpy as np
from .mapping import map_joints, mix
from ....inputs.pnlink.state import read_state, fresh
from ....core.runtime_timing import wait_event
from ..wire import pack_pose_message


class PoseReader:
    def __init__(self, path, clock=time.monotonic):
        self.path, self.clock = Path(path), clock
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name='pnlink-mapping')
        self.receiver_id = None
        self.frame_id = None
        self.source_timestamp = None
        self.regressed = False
        self.prev = None
        self.next_stamp = None
        self.frames = deque(maxlen=5)
        self.fields = self.packet = self.updated = None
        self.raw = {}
        self.reason = '等待 PN Link 身体数据'
        self.mapping_ms = 0.

    def invalidate(self, reason):
        self.prev = self.next_stamp = self.fields = self.packet = self.updated = None
        self.frames.clear()
        self.reason = reason

    def feed(self, data):
        with self.lock:
            now = self.clock()
            generation = data.get('receiver_id')
            if not isinstance(generation, str) or not generation:
                raise ValueError('PN Link 接收会话无效')
            if generation != self.receiver_id:
                self.invalidate('PN Link 接收器已重启')
                self.receiver_id = generation
                self.frame_id = self.source_timestamp = None
                self.regressed = False
            self.raw = data
            if not fresh(data, now):
                self.invalidate(data.get('error') or 'PN Link 未采集、校准中或数据已过期')
                return
            frame, stamp = data.get('frame_id'), data.get('source_timestamp')
            received = data.get('received_monotonic')
            if type(frame) is not int or frame <= 0 or type(stamp) not in (int, float) or not math.isfinite(stamp):
                raise ValueError('PN Link 帧标识无效')
            if self.regressed:
                return
            if (self.frame_id is not None and frame < self.frame_id or
                    self.source_timestamp is not None and stamp < self.source_timestamp or
                    self.prev is not None and received < self.prev[0]):
                self.regressed = True
                self.invalidate('PN Link 时间或帧号回退，需重建接收会话')
                return
            if frame == self.frame_id or (stamp > 0 and stamp == self.source_timestamp):
                return  # File rewrites and repeated SDK timestamps never refresh freshness.
            started = time.perf_counter()
            mapped = map_joints(data['joints'])
            if self.prev and np.dot(self.prev[1]['body_quat_w'], mapped['body_quat_w']) < 0:
                mapped['body_quat_w'] *= -1
            self.mapping_ms = (time.perf_counter()-started)*1000
            self.frame_id, self.source_timestamp = frame, stamp
            if self.prev and received-self.prev[0] > .25:
                self.invalidate('PN Link 人体帧间隔过大')
            if self.prev is None:
                self.prev = received, mapped
                self.next_stamp = received
                return
            old_time, old = self.prev
            if received <= old_time:
                return
            # At most 13 samples; no unbounded backlog or wall-clock catch-up.
            while self.next_stamp <= received:
                if self.next_stamp >= old_time:
                    self.frames.append(mix(old, mapped, (self.next_stamp-old_time)/(received-old_time)))
                self.next_stamp += .02
            self.prev = received, mapped
            self.updated = received
            if len(self.frames) == 5:
                self.fields = {k: np.stack([row[k] for row in self.frames]) for k in mapped}
                self.fields['frame_index'] = np.arange(frame*5, frame*5+5, dtype=np.int64)
                self.fields['timestamp_monotonic'] = np.array([now], dtype=np.float64)
                self.packet = pack_pose_message(self.fields, version=3)
                self.reason = 'PN Link SMPL · 50 Hz · 5 帧窗口'

    def snapshot(self):
        with self.lock:
            ready = bool(self.packet and self.updated is not None and 0 <= self.clock()-self.updated <= .25)
            return dict(ready=ready, packet=self.packet if ready else None,
                        fields=self.fields if ready else None, receiver_id=self.receiver_id,
                        reason=self.reason if ready else '等待有效 PN Link 人体数据',
                        raw=self.raw, mapping_ms=self.mapping_ms)

    def run(self):
        while not self.stop.is_set():
            try:
                self.feed(read_state(self.path))
            except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
                with self.lock:
                    self.invalidate('PN Link 接收或映射失败：' + str(exc))
            # JP5/Python 3.8 Event.wait can stall across a wall-clock rollback.
            # Keep observing new receiver generations even after an offline boot.
            wait_event(self.stop, .01)
