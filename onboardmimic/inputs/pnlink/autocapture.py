"""Opt-in device capture after explicit service start; never robot authority.

Only our own idempotent start-capture request may trigger SDK reconnection.
Manual stop/calibration suspend automation until manual start or service restart.
"""
import secrets


class CaptureReconnect(RuntimeError):
    pass


class AutoCapture:
    def __init__(self, clock, retry_seconds=5., command_timeout=20.):
        self.clock = clock
        self.retry_seconds = retry_seconds
        self.command_timeout = command_timeout
        self.next_attempt = clock()+2.
        self.suspended = False
        self.attempts = 0
        self.error = ''

    def manual(self, action):
        self.suspended = action != 'start_capture'
        self.next_attempt = self.clock()+self.retry_seconds

    def tick(self, receiver):
        now = self.clock()
        data = receiver.data
        state = data['auto_capture'] = dict(enabled=True, suspended=self.suspended,
                                             attempts=self.attempts, error=self.error)
        if self.suspended:
            state['state'] = 'manual_control'
            return
        command = data.get('command', {})
        if receiver.pending:
            state['state'] = 'waiting_command'
            if (command.get('automatic') is True and receiver.pending == 'start_capture'
                    and now-command['started_monotonic'] > self.command_timeout):
                raise CaptureReconnect('自动开始采集未获明确应答，重建只接收SDK；不恢复机器人接管')
            return
        stamp = data.get('received_monotonic')
        if stamp is not None and 0 <= now-stamp <= .25 and data.get('capture_state') == 'capturing':
            state['state'] = 'receiving'
            self.error = ''
            self.next_attempt = now+self.retry_seconds
            return
        state['state'] = 'waiting_device'
        if now < self.next_attempt:
            return
        self.next_attempt = now+self.retry_seconds
        self.attempts += 1
        try:
            receiver.submit(dict(action='start_capture', receiver_id=receiver.id,
                                 request_id='auto-'+secrets.token_hex(12)), automatic=True)
            self.error = ''
        except (ValueError, RuntimeError) as exc:
            self.error = str(exc)
