"""Gateway-owned PICO authority and acknowledged source handoffs. No I/O."""
import secrets
import json
import logging

class PicoControl:
    def pico_owned(self):
        return bool(self.pico_session and self.pico_session.active)

    def revoke_pico(self,reason=''):
        if self.pico_session:self.pico_session.revoke(reason)

    def pico_q(self):
        t=self.telemetry()
        if not self.robot.safe(t) or t.get('age_ms',999)>250:
            raise ValueError('PICO 切换需要新鲜的机器人关节反馈')
        return t['q']

    def start_pico_task(self,mode,aligned=False,origin='browser'):
        p=self.pico_session
        if p is None or not p.ready():raise ValueError('PICO 身体和双手柄尚未就绪')
        if mode not in ('pose','planner','frozen','vr3pt'):raise ValueError('未知 PICO 子模式')
        if self.phase!='active' or not self.alive() or self.mode_pending or self.task_active():
            raise ValueError('请先启动控制器并等待当前任务完成')
        if not self.pico_owned() and aligned is not True:
            raise ValueError('请确认头显已标定，操作者已与机器人姿态对齐')
        self.pico_q()
        if not self.policy_feedback or self.clock()-self.policy_feedback[0]>.25:
            raise ValueError('控制器模式反馈尚未就绪')
        new_session=not self.pico_owned()
        self.revoke_pnlink('切换到 PICO', open_hand=True)
        if self.input_mode in ('joint_playback','smpl_playback'):self.playback.end()
        self.motion_task=dict(id=secrets.token_hex(12),kind='pico',stage='waiting_planner',
                              target=mode,error='',origin=origin,new_session=new_session,
                              deadline=self.clock()+10.)
        self.begin_mode('planner','PICO 切换：等待零速 Planner 确认并站稳')
        self.revision+=1

    def tick_pico_task(self,now):
        task=self.motion_task;p=self.pico_session
        if self.phase!='active' or not self.alive() or now>task['deadline'] or not p.ready():
            self.cancel_motion_task('PICO 切换超时、控制器停止或追踪失效')
            self.revoke_pico('切换未完成，请人工重新进入')
            return
        if task['stage']=='starting':
            if not self.mode_pending and p.active:
                task['stage']='finished';self.revision+=1
            elif not p.active:
                task.update(stage='error',error=self.mode_error or 'PICO 接管失败')
            return
        if not self.stationary(self.telemetry()):return
        if not self.policy_feedback or now-self.policy_feedback[0]>.25:return
        if self.policy_feedback[1].get('mode')!='planner' or self.policy_feedback[1].get('playing') is not True:return
        try:
            p.enter(task['target'],self.pico_q(),new_session=task['new_session'])
            if task.get('origin')=='browser' and not task['new_session']:
                p.reset_hands()  # Explicit re-entry can rearm a faulted hand session.
            self.begin_mode('pose' if task['target']=='pose' else 'planner')
            task['stage']='starting';task['deadline']=now+3.
        except (ValueError,RuntimeError) as exc:
            self.revoke_pico(str(exc));task.update(stage='error',error=str(exc))
            self.begin_mode('planner',str(exc))

    def tick_pico(self,now):
        if not self.pico_owned():return
        with self.pico_session.reader.lock:
            self._tick_pico_locked(now)

    def _tick_pico_locked(self,now):
        p=self.pico_session
        tracking=p.reader.snapshot()
        if p.released and tracking.get('controllers_ready') and 'stop' in p.buttons(tracking):
            self.halt();return
        if not self.task_active() and not self.mode_pending:
            expected=2 if p.mode=='pose' else 1 if p.mode=='vr3pt' else 0
            if (not self.policy_feedback or now-self.policy_feedback[0]>.5 or
                self.policy_feedback[1].get('encoder_mode')!=expected or
                self.policy_feedback[1].get('mode')!=('pose' if p.mode=='pose' else 'planner') or
                self.policy_feedback[1].get('playing') is not True):
                self.revoke_pico('PICO 控制器模式反馈失效')
                self.begin_mode('planner','PICO 控制器模式反馈失效')
                return
        was_recovering=p.recovering
        state=p.update_tracking(now,tracking,allow_hold=not self.task_active() and not self.mode_pending)
        if state=='holding':
            if not was_recovering:
                logging.getLogger('runtime_audit').warning(json.dumps(dict(
                    event='pico_tracking_hold',mode=p.mode,window_seconds=p.recovery_seconds,
                    tracking=p.status(tracking)['tracking'],
                    controllers_ready=tracking.get('controllers_ready'),
                    controller_errors=tracking.get('controller_errors',{})),ensure_ascii=False))
            return
        if state=='lost':
            # Preserve the failing input evidence before revoke clears the session.
            # No pose payload or actuator targets belong in this diagnostic.
            snapshot=p.status(tracking)
            logging.getLogger('runtime_audit').warning(json.dumps(dict(
                event='pico_tracking_lost',expected_receiver_id=p.receiver_id,
                mode=p.mode,tracking=snapshot.get('tracking'),
                controllers_ready=snapshot.get('controllers_ready'),
                controller_age_ms=snapshot.get('controller_age_ms'),
                controller_errors=snapshot.get('controller_errors')),ensure_ascii=False))
            self.cancel_motion_task('PICO 身体或手柄断流')
            reason=p.recovery_reason or 'PICO 身体或手柄断流，需人工重新进入'
            self.revoke_pico(reason)
            self.begin_mode('planner',reason)
            return
        if state=='resumed':
            old_hand_session=p.hand_session
            p.reset_hands()
            # An optional hand fault still needs explicit re-entry; network
            # recovery must not silently rearm an already faulted hand output.
            if self.pico_hand_error:p.hand_session=old_hand_session
            logging.getLogger('runtime_audit').info(json.dumps(dict(event='pico_tracking_resumed',mode=p.mode)))
        if self.task_active() or self.mode_pending:return
        event=p.events()
        if event=='stop':self.halt();return
        if event in ('lost','resume_rejected'):
            reason='PICO 追踪失效' if event=='lost' else '暂停恢复姿态偏差过大，请对齐后重新进入'
            self.revoke_pico(reason);self.begin_mode('planner',reason);return
        if event:
            try:self.start_pico_task(event,origin='pico')
            except (ValueError,RuntimeError) as exc:
                self.revoke_pico(str(exc));self.begin_mode('planner',str(exc))

    def pico_reference(self,now):
        if not self.pico_owned():return None
        # The reader may go stale between the normal tick and publication.
        # Make that decision and reference from one locked reader generation.
        with self.pico_session.reader.lock:
            self.tick_pico(now)
            if (self.task_active() and self.motion_task.get('kind')=='pico' and
                self.motion_task['stage']!='starting'):return None
            if not self.pico_owned():return None
            try:return self.pico_session.reference(now)
            except (ValueError,TypeError,KeyError,RuntimeError) as exc:
                self.revoke_pico(str(exc));self.begin_mode('planner','PICO 参考生成失败：'+str(exc))
                return None
