"""PN authority and acknowledged handoffs; device I/O runs outside control lock."""
import secrets
import threading


class PNLinkControl:
    def pnlink_busy(self):
        request = self.pnlink_command
        if not request or request['stage'] in ('error', 'finished'):
            return False
        if request['stage'] == 'sending':
            return True
        raw = self.pnlink_session.reader.snapshot().get('raw', {})
        if raw.get('receiver_id') != request.get('receiver_id'):
            return False
        result = raw.get('command', {})
        return not (result.get('request_id') == request['request_id'] and
                    result.get('stage') in ('succeeded', 'failed'))

    def pnlink_owned(self):
        return bool(self.pnlink_session and self.pnlink_session.active)

    def tracking_owned(self):
        return self.pico_owned() or self.pnlink_owned()

    def active_pose_reader(self):
        return self.pnlink_session.reader if self.pnlink_owned() else self.pose_reader

    def revoke_tracking(self, reason='', open_hand=False):
        was_pico = self.pico_owned()
        self.revoke_pico(reason)
        self.revoke_pnlink(reason, open_hand=open_hand and (not was_pico or bool(self.pnlink_hands and self.pnlink_hands.finishing())))

    def revoke_pnlink(self, reason='', open_hand=False):
        if self.pnlink_hands:
            self.pnlink_hands.revoke(reason, open_hand=open_hand)
        if self.pnlink_session:
            self.pnlink_session.revoke(reason)

    def start_pnlink_task(self, aligned):
        p = self.pnlink_session
        if self.pnlink_hands and (self.pnlink_hands.sampling() or self.pnlink_hands.finishing()):
            raise ValueError('请等待手部采样或结束清零完成')
        if not p or self.pnlink_busy() or not p.ready() or not p.calibrated():
            raise ValueError('PN Link 未就绪，请采集、预览并确认当前穿戴校准')
        if aligned is not True:
            raise ValueError('请确认操作者与机器人姿态对齐')
        if self.phase != 'active' or not self.alive() or self.mode_pending or self.task_active():
            raise ValueError('请先启动控制器并等待当前切换完成')
        telemetry = self.telemetry()
        if not self.robot.safe(telemetry) or telemetry.get('age_ms',999) > 250:
            raise ValueError('PN Link 切换需要新鲜的机器人关节反馈')
        if not self.policy_feedback or self.clock()-self.policy_feedback[0] > .25:
            raise ValueError('控制器模式反馈尚未就绪')
        if self.input_mode in ('joint_playback', 'smpl_playback'):
            self.playback.end()
        self.revoke_tracking('切换到 PN Link')
        self.motion_task = dict(id=secrets.token_hex(12), kind='pnlink', target='pnlink',
                                stage='waiting_planner', error='', deadline=self.clock()+10.)
        self.begin_mode('planner', 'PN Link 切换：等待零速 Planner 确认并站稳')

    def dispatch_pnlink_command(self, action):
        if action == 'calibrate' and self.pnlink_hands:
            self.pnlink_hands.invalidate_calibration()
        service_action = action in ('start_receiver', 'stop_receiver')
        if self.pnlink_command and self.pnlink_command['stage'] == 'sending':
            raise ValueError('PN Link 命令发送中，请等待状态')
        p = self.pnlink_session
        if p is None or (not service_action and self.pnlink_socket is None):
            raise ValueError('PN Link 接收服务未配置')
        generation = p.reader.snapshot().get('receiver_id')
        if not generation and not service_action:
            raise ValueError('PN Link 接收会话未就绪')
        if not service_action or action == 'stop_receiver':p.clear_calibration()
        request = dict(action=action, kind='service' if service_action else 'sdk',
                       request_id=secrets.token_hex(12), receiver_id=generation, stage='sending', error='')
        self.pnlink_command = request
        def send():
            try:
                if service_action:
                    self.backend.pnlink_service(action)
                else:
                    self.backend.pnlink_command(self.pnlink_socket, action, generation, request['request_id'])
                result = dict(stage='finished' if service_action else 'accepted')
            except (OSError, ValueError, RuntimeError) as exc:
                result = dict(stage='unknown' if isinstance(exc, OSError) else 'error', error=str(exc))
            with self.lock:
                request.update(result)
                self.revision += 1
        threading.Thread(target=send, daemon=True, name='pnlink-command').start()

    def pnlink_request(self, x):
        p = self.pnlink_session
        if not p:
            raise ValueError('PN Link 未配置')
        action = x.get('action')
        if action in ('configure_hands', 'capture_hand_open', 'capture_hand_closed',
                      'capture_hand_finger', 'confirm_hand_calibration'):
            h = self.pnlink_hands
            if h is None:
                raise ValueError('端侧手部功能未配置')
            h.update()
            if action == 'configure_hands':
                h.configure(x.get('hand_mode'), x.get('hand_closure'), self.pnlink_owned() or self.mode_pending or self.task_active(), hand=x.get('hand'))
            else:
                if self.pnlink_owned() or self.mode_pending or self.task_active() or self.pnlink_busy() or h.finishing():
                    raise ValueError('请先退出 PN Link 遥操并等待设备操作完成')
                if action == 'confirm_hand_calibration':
                    h.confirm_hands(x.get('hand', 'right'), x.get('calibration_confirmed'))
                else:
                    h.capture_hands(x.get('hand', 'right'), action[len('capture_hand_'):], x.get('finger'))
            return
        if self.pnlink_hands and self.pnlink_hands.sampling():
            raise ValueError('请先等待手部采样完成')
        if action == 'confirm_calibration':
            if self.pnlink_busy():
                raise ValueError('设备命令尚未有明确结果，请等待后再确认校准')
            if x.get('calibration_confirmed') is not True:
                raise ValueError('请确认当前穿戴与人体预览正常')
            p.confirm()
            return
        if action not in ('start_receiver', 'stop_receiver', 'start_capture', 'stop_capture', 'calibrate'):
            raise ValueError('未知 PN Link 操作')
        if action == 'start_receiver':
            self.dispatch_pnlink_command(action)
            return
        if self.pnlink_owned():
            if self.mode_pending or self.task_active():
                raise ValueError('请等待当前模式切换完成')
            self.revoke_pnlink('设备操作前退出动捕遥操', open_hand=True)
            self.motion_task = dict(id=secrets.token_hex(12), kind='pnlink_device', target=action,
                                    stage='waiting_planner', error='', deadline=self.clock()+10.)
            self.begin_mode('planner', '等待零速 Planner 确认和站稳后操作动捕设备')
        else:
            self.dispatch_pnlink_command(action)

    def tick_pnlink_task(self, now):
        task, p = self.motion_task, self.pnlink_session
        if self.phase != 'active' or not self.alive() or now > task['deadline']:
            self.cancel_motion_task('PN Link 切换超时或控制器停止')
            return
        if task['kind'] == 'pnlink' and (not p.ready() or not p.calibrated()):
            self.cancel_motion_task('PN Link 数据或校准确认失效')
            return
        if task['stage'] == 'starting':
            if not self.pnlink_owned():
                task.update(stage='error', error=self.mode_error or 'PN Link 接管失败')
            elif not self.mode_pending:
                task['stage'] = 'finished'
                self.revision += 1
            return
        if not self.stationary(self.telemetry()) or not self.feedback_ready():
            return
        if not self.policy_feedback or now-self.policy_feedback[0] > .25:
            return
        ack = self.policy_feedback[1]
        if ack.get('mode') != 'planner' or ack.get('encoder_mode') != 0 or ack.get('playing') is not True:
            return
        try:
            if task['kind'] == 'pnlink_device':
                self.dispatch_pnlink_command(task['target'])
                task['stage'] = 'finished'  # Handoff finished; SDK result remains separate.
            else:
                p.enter()
                if self.pnlink_hands:
                    self.pnlink_hands.enter()
                self.begin_mode('pose')
                task.update(stage='starting', deadline=now+3.)
        except (OSError, ValueError, RuntimeError) as exc:
            self.revoke_pnlink(str(exc))
            task.update(stage='error', error=str(exc))
            self.begin_mode('planner', str(exc))

    def tick_pnlink(self, now):
        if not self.pnlink_owned():
            return
        p = self.pnlink_session
        if not p.ready() or not p.calibrated() or p.reader.snapshot()['receiver_id'] != p.receiver_id:
            reason = 'PN Link 数据或会话失效，需人工重新进入'
            self.cancel_motion_task(reason)
            self.revoke_pnlink(reason)
            self.begin_mode('planner', reason)

    def pnlink_reference(self, now):
        try:
            packet = self.pnlink_session.reference(now)
            if packet is None:
                raise ValueError('PN Link 参考失效')
            return packet
        except (ValueError, TypeError, KeyError, RuntimeError) as exc:
            self.revoke_pnlink(str(exc))
            self.begin_mode('planner', 'PN Link 参考生成失败：' + str(exc))
            return None
