"""Bounded local snapshots and command IPC, without robot-control dependencies."""
import json
import math
import os
from pathlib import Path
import socket
import time

MAX_SNAPSHOT = 256 * 1024


def read_state(path):
    path = Path(path)
    if path.stat().st_size > MAX_SNAPSHOT:
        raise ValueError('PN Link 状态文件过大')
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or data.get('protocol') != 1:
        raise ValueError('PN Link 状态版本无效')
    return data


def fresh(data, now=None):
    now = time.monotonic() if now is None else now
    stamp = data.get('received_monotonic')
    return (data.get('data_source') == 'sdk' and data.get('receiver_running') is True
            and type(stamp) in (int, float) and math.isfinite(stamp)
            and 0 <= now - stamp <= .25 and data.get('capture_state') == 'capturing'
            and data.get('command', {}).get('stage') not in ('running', 'unknown'))


def atomic_write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    tmp = path.with_name(path.name + '.tmp')
    with tmp.open('w') as out:
        os.chmod(tmp, 0o600)
        json.dump(data, out, ensure_ascii=False, allow_nan=False)
    tmp.replace(path)


def command(path, action, receiver_id, request_id):
    """One bounded request; never retry an uncertain device operation."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
        conn.settimeout(.3)
        conn.connect(str(path))
        conn.sendall(json.dumps(dict(action=action, receiver_id=receiver_id,
                                    request_id=request_id)).encode() + b'\n')
        raw = bytearray()
        while b'\n' not in raw:
            block = conn.recv(4096)
            if not block:
                raise RuntimeError('PN Link 命令应答中断；请查看状态，勿自动重试')
            raw.extend(block)
            if len(raw) > 8192:
                raise ValueError('PN Link 命令应答过大')
    result = json.loads(raw)
    if result.get('error'):
        raise ValueError(result['error'])
    return result
