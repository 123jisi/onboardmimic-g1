"""User-service lifecycle only, invoked by authenticated gateway actions."""
import subprocess

NAME = 'onboardmimic-pnlink-receiver.service'


def change(action, runner=subprocess.run):
    verbs = {'start_receiver': 'start', 'stop_receiver': 'stop'}
    if action not in verbs:
        raise ValueError('未知动捕接收服务操作')
    try:
        result = runner(['systemctl', '--user', verbs[action], NAME],
                        capture_output=True, text=True, timeout=12)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError('动捕接收服务未确认，请查看服务状态：' + str(exc)) from exc
    if result.returncode:
        raise RuntimeError('动捕接收服务操作失败：' + result.stderr.strip())


def status(runner=subprocess.run):
    try:
        result = runner(['systemctl', '--user', 'show', NAME, '--no-pager',
                         '-p', 'ActiveState', '-p', 'UnitFileState'],
                        capture_output=True, text=True, timeout=1)
        values = dict(row.split('=', 1) for row in result.stdout.splitlines() if '=' in row)
        return dict(active_state=values.get('ActiveState', 'unknown'),
                    unit_file_state=values.get('UnitFileState', 'unknown'))
    except (OSError, subprocess.TimeoutExpired):
        return dict(active_state='unknown', unit_file_state='unknown')
