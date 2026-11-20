"""Runtime-owned operator authority and cancellable file handoff transactions.

Browser leases authorize future starts; they are never the playback heartbeat.
All methods run under Lifecycle.lock except the background file preparation.
"""
import secrets
import logging
import json
import threading

TERMINAL = {'finished', 'error', 'cancelled'}


class Operations:
    def init_operations(self):
        self.operator = None
        self.operator_generation = 0
        self.operator_deadline = 0.
        self.revision = 0
        self.task_requests = {}
        self.motion_task = None
        self.prepared_clip = None
        self.library = None
        self.conversion = None
        self.conversion_cancel = None
        self.conversion_thread = None

    def operator_status(self):
        return dict(owner=self.operator, generation=self.operator_generation,
                    remaining_ms=max(0, int((self.operator_deadline-self.clock())*1000)),
                    revision=self.revision)

    def operator_request(self, x):
        actor = x.get('actor')
        if not isinstance(actor, str) or not 16 <= len(actor) <= 100:
            raise ValueError('无效操作端身份')
        action = x.get('action')
        if action in ('acquire', 'takeover'):
            if self.operator and self.operator != actor and action != 'takeover':
                raise ValueError('另一操作端持有操作权，请显式接管')
            if self.operator != actor:
                self.cancel_motion_task('操作权已变更', only_unstarted=True)
                self.operator_generation += 1
                self.revision += 1
            pending_tracking = self.task_active() and self.motion_task.get('kind') in ('pico','pnlink','pnlink_device')
            if action=='takeover' and (self.tracking_owned() or pending_tracking):
                self.cancel_motion_task('手机主动接管', open_hand=True)
                self.revoke_tracking('手机主动接管', open_hand=True)
                if self.phase=='active':self.begin_mode('planner','手机主动接管，遥操已退出')
            self.operator, self.operator_deadline = actor, self.clock()+5.
        elif action == 'authorize':
            self.require_operator(x)
            if self.conversion and self.conversion['stage'] == 'running':
                raise ValueError('请先完成或取消转换任务')
        elif action in ('heartbeat', 'release'):
            self.require_operator(x, revision=False)
            if action == 'release':
                self.expire_operator('操作权已释放')
            else:
                self.operator_deadline = self.clock()+5.
        else:
            raise ValueError('未知操作权操作')

    def expire_operator(self, reason):
        self.operator = None
        self.operator_deadline = 0.
        self.operator_generation += 1
        self.revision += 1
        self.cancel_motion_task(reason, only_unstarted=True)

    def require_operator(self, x, revision=True):
        if self.operator:
            if x.get('actor') != self.operator or x.get('operator_generation') != self.operator_generation:
                raise ValueError('请先取得操作权')
        elif x.get('actor') is not None:
            raise ValueError('请先取得操作权')
        if x.get('actor') is not None and revision and x.get('expected_revision') != self.revision:
            raise ValueError('控制状态已改变，请刷新后重试')

    def task_active(self):
        return self.motion_task and self.motion_task['stage'] not in TERMINAL

    def cancel_motion_task(self, reason='已取消换播', only_unstarted=False, open_hand=False):
        if self.task_active():
            if only_unstarted and self.motion_task.get('origin')=='pico':return
            if self.motion_task.get('kind') in ('pico','pnlink','pnlink_device'):
                if only_unstarted and self.motion_task['stage']=='starting':return
                self.revoke_tracking(reason, open_hand=open_hand)
                if self.phase=='active':self.begin_mode('planner',reason)
            if self.motion_task['stage'] == 'starting':
                if only_unstarted:
                    return
                if self.phase == 'active' and self.input_mode in ('joint_playback','smpl_playback'):
                    self.playback.end()
                    self.begin_mode('planner', reason)
            self.motion_task.update(stage='cancelled', error=reason)
            self.prepared_clip = None
            self.revision += 1

    def start_motion_task(self, x, autoplay):
        request_id = x.get('request_id')
        if request_id is not None and (not isinstance(request_id,str) or not 1 <= len(request_id) <= 100):
            raise ValueError('无效任务标识')
        identity = (self.operator_generation, self.operator, request_id)
        if request_id and identity in self.task_requests:
            return
        if self.task_active():
            if x.get('request_id') and x.get('request_id') == self.motion_task.get('request_id'):
                return
            raise ValueError('已有动作任务，请等待或取消')
        if not self.playback or not self.library:
            raise ValueError('文件播放未启用')
        if self.conversion and self.conversion['stage'] == 'running':
            raise ValueError('转换正在执行')
        if self.mode_pending or (self.input_mode == 'pose' and not (autoplay and self.tracking_owned())):
            raise ValueError('请先退出遥操并等待模式确认')
        if self.tracking_owned() and not autoplay:
            raise ValueError('遥操期间请使用切换并播放；只加载不会退出遥操')
        if self.phase == 'active':
            if not self.alive() or not self.robot.safe(self.telemetry()):
                raise ValueError('控制器或反馈未就绪')
        elif autoplay or self.phase not in ('stopped', 'error') or self.alive():
            raise ValueError('当前状态不允许加载或播放')
        checks = x.get('entry_checks', 'on')
        if checks not in ('on', 'off'):
            raise ValueError('entry_checks 必须为 on 或 off')
        spec = self.library.load_spec(x)
        task = dict(id=secrets.token_hex(12), request_id=x.get('request_id'),
                    stage='checking', target=spec['motion'], error='',
                    autoplay=autoplay, entry_checks=checks,
                    actor=self.operator, operator_generation=self.operator_generation,
                    deadline=self.clock()+60.)
        self.motion_task = task
        if request_id:
            if len(self.task_requests) >= 128:
                self.task_requests.pop(next(iter(self.task_requests)))
            self.task_requests[identity] = task
        self.prepared_clip = None
        self.revision += 1
        def prepare():
            try:
                clip = self.backend.prepare_clip(self.motion_root, spec,
                    lambda: self.motion_task is not task or task['stage'] in TERMINAL)
                if autoplay and checks == 'on' and hasattr(self.backend, 'validate_static_first_frame'):
                    self.backend.validate_static_first_frame(clip)
                with self.lock:
                    if self.motion_task is task and task['stage'] == 'checking':
                        self.prepared_clip = clip
                        task['stage'] = 'ready'
                        meta=clip.metadata()
                        task['clip_identity']={k:meta[k] for k in ('name','frames','fps','hashes','cache_sha256','source_sha256') if k in meta}
                        logging.getLogger('control_audit').info(json.dumps(dict(event='motion_prepared',monotonic=self.clock(),task_id=task['id'],entry_checks=task['entry_checks'],clip=task['clip_identity'])))
            except Exception as exc:
                with self.lock:
                    if self.motion_task is task and task['stage'] not in TERMINAL:
                        task.update(stage='error', error=str(exc))
                        self.revision += 1
        threading.Thread(target=prepare, daemon=True, name='motion-prepare').start()

    def tick_operations(self, now):
        if self.operator and now >= self.operator_deadline:
            self.expire_operator('操作端失联，未开始的任务已取消')
        task = self.motion_task
        if not self.task_active():
            return
        if task.get('kind') in ('pnlink','pnlink_device'):
            self.tick_pnlink_task(now)
            return
        if task.get('kind')=='pico':
            self.tick_pico_task(now)
            return
        # Once a play was actually issued it belongs to the local player, not
        # to the browser. Mode failures remain owned by the usual lifecycle.
        if task['stage'] == 'starting':
            if self.input_mode == 'planner':
                task.update(stage='error', error=self.mode_error or '播放接管失败')
            elif not self.mode_pending and self.playback.confirmed:
                task['stage'] = 'finished'
            return
        if now > task['deadline']:
            self.cancel_motion_task('动作任务超时')
            return
        if task['stage'] == 'checking':
            return
        if self.phase in ('stopped', 'error') and not self.alive():
            if task['autoplay']:
                self.cancel_motion_task('控制器已停止')
                return
            if task['stage'] == 'ready':
                self.playback.load(self.prepared_clip)
                self.prepared_clip = None
                self.playback_entry_checks = None
                task['stage'] = 'finished'
                self.revision += 1
            return
        if self.phase != 'active' or not self.alive():
            self.cancel_motion_task('控制状态已改变')
            return
        if task['stage'] == 'ready':
            # Preflight succeeded. Only now interrupt the old source.
            if self.tracking_owned():
                self.revoke_tracking('已切换文件播放', open_hand=True)
                self.begin_mode('planner','切换文件：等待零速 Planner 确认')
            elif self.input_mode in ('joint_playback', 'smpl_playback'):
                self.playback.end()
                self.begin_mode('planner', '换播：等待零速 Planner 确认')
            elif self.input_mode != 'planner':
                self.cancel_motion_task('输入来源已改变')
                return
            else:
                self.lease.stop()
                self.still_since = None
            task.update(stage='waiting_planner', deadline=now+10.)
        if task['stage'] != 'waiting_planner' or not self.stationary(self.telemetry()):
            return
        if not self.feedback_ready() or not self.policy_feedback or now-self.policy_feedback[0] > .25:
            return
        if self.policy_feedback[1].get('mode') != 'planner' or self.policy_feedback[1].get('playing') is not True:
            return
        clip = self.prepared_clip
        try:
            if task['autoplay'] and task['entry_checks'] == 'on':
                self.backend.validate_first_frame(clip, self.telemetry(), self.latest)
            self.playback.load(clip)
            self.prepared_clip = None
            self.playback_entry_checks = None
            if task['autoplay']:
                self.playback.start(now)
                self.playback_entry_checks = task['entry_checks']
                self.begin_mode(clip.mode)
                task['stage'] = 'starting'
            else:
                task['stage'] = 'finished'
            self.revision += 1
        except Exception as exc:
            self.prepared_clip = None
            task.update(stage='error', error=str(exc))
            self.revision += 1

    def library_request(self, route, x):
        if not self.library:
            raise ValueError('动作库未启用')
        if route == '/api/motions':
            if x.get('action') != 'refresh': raise ValueError('未知动作库操作')
            if self.library.state == 'scanning': return
            self.library.refresh()
            return
        if x.get('action') == 'cancel':
            if self.conversion and self.conversion['stage'] == 'running':
                self.conversion_cancel.set()
            return
        if x.get('action') not in ('convert', 'register'):
            raise ValueError('未知转换操作')
        if self.phase not in ('stopped', 'error') or self.alive() or self.task_active():
            raise ValueError('请在控制器停止时转换；已转换动作可在运行中换播')
        if self.conversion and self.conversion['stage'] == 'running':
            raise ValueError('已有转换任务')
        spec = self.library.conversion_spec(x)
        task = dict(id=secrets.token_hex(12), stage='running', motion_id=x.get('motion_id'),
                    error='', result=None)
        cancel = threading.Event()
        self.conversion, self.conversion_cancel = task, cancel
        self.revision += 1
        def convert():
            try:
                result = self.library.convert(spec, cancel.is_set)
                with self.lock:
                    replaced=result.get('replaced_motions',[])
                    if self.playback and self.playback.clip and self.playback.clip.metadata().get('name') in replaced:
                        # Replacing a file also invalidates its old in-memory clip.
                        # Stay unloaded; never auto-load or start the new motion.
                        self.playback.clip=None
                        self.playback.position=0.
                        self.playback.end()
                        self.playback_entry_checks=None
                        self.prepared_clip=None
                    task.update(stage='finished', result=result)
                    self.revision += 1
                self.library.refresh()
            except Exception as exc:
                with self.lock:
                    task.update(stage='cancelled' if cancel.is_set() else 'error', error=str(exc))
                    self.revision += 1
        self.conversion_thread = threading.Thread(target=convert, daemon=True, name='motion-import')
        self.conversion_thread.start()
