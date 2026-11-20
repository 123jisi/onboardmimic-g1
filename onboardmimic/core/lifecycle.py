"""Measured-feedback lifecycle shared by physical deployment and isolated tests.
Only launch/stop adapters own processes; this module never imports a robot SDK.
"""
import json
import logging
import math
import secrets
import time
from .gateway import Gateway, PlannerLease
from .operations import Operations
from .pico_control import PicoControl
from .pnlink_control import PNLinkControl


class Lifecycle(PNLinkControl, PicoControl, Operations, Gateway):
    def __init__(self, baseline, launch, read_log, guard, physical=False, clock=time.monotonic, *, robot, backend, require_continuity=True):
        super().__init__(baseline, clock)
        self.robot, self.backend = robot, backend
        self.lease = PlannerLease(min(.5, backend.max_speed), max_speed=backend.max_speed)
        self.require_continuity = require_continuity
        self.launch, self.read_log, self.guard = launch, read_log, guard
        self.physical = physical
        self.latest = {}
        self.process = None
        self.failure = ''
        self.phase_deadline = 0.
        self.still_since = None
        self.start_pending = False
        self.pose_reader = None
        self.pico_session = None
        self.pnlink_session = None
        self.pnlink_socket = None
        self.pnlink_command = None
        self.pnlink_hands = None
        self.pico_hand_link = None
        self.pico_hand_error = ''
        self.input_mode = 'planner'
        self.mode_pending = False
        self.mode_requested_at = 0.
        self.mode_error = ''
        self.policy_feedback = None
        self.playback = None
        self.motion_root = None
        self.playback_entry_checks = None
        self.init_operations()

    def ingest(self, topic, value):
        with self.lock:
            self.robot.ingest(topic, value, self.clock())
            normalized = self.backend.decode_status(topic, value)
            if normalized is not None:
                self.policy_feedback = (self.clock(), normalized)
            if topic in self.robot.state_topics:
                self.latest[topic] = (self.clock(), value)

    def telemetry(self):
        return self.robot.telemetry(self.latest, self.baseline, self.clock())

    def feedback_ready(self):
        return not self.require_continuity or self.robot.feedback_ready(self.latest,self.clock())

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def halt(self, error='', open_hand=False):
        # Caller holds lock: a queued start cannot survive this generation change.
        self.cancel_motion_task(error or "控制器已停止", open_hand=open_hand)
        self.revoke_tracking(error or "控制器已停止", open_hand=open_hand)
        self.epoch = secrets.token_hex(16)
        self.sequence.clear()
        self.lease.stop()
        if self.playback:
            self.playback.end('error' if error else 'loaded', error)
        self.input_mode = 'planner'
        self.mode_pending = False
        # A switch reason belongs to the retired controller session. Preserve
        # real stop/fault detail in failure and audit/task history instead.
        self.mode_error = ''
        self.mode_requested_at = 0.
        self.last_packet = None
        self.start_pending = False
        self.still_since = None
        self.phase = 'error' if error else 'stopped'
        self.failure = error
        if self.alive():
            self.process.kill()
            try:
                self.process.wait(timeout=3)
            except Exception as exc:
                self.phase = 'error'
                self.failure = '控制进程退出未确认：' + str(exc)
                raise RuntimeError(self.failure) from exc

    def tick(self):
        with self.lock:
            now = self.clock()
            if self.pnlink_hands:
                try:
                    self.pnlink_hands.update()
                except Exception as exc:
                    self.pnlink_hands.fail('手部处理异常，已停用手部：'+str(exc))
            if self.operator and now >= self.operator_deadline:
                self.expire_operator("操作端失联，未开始的任务已取消")
            if self.phase in ('stopped', 'error'):
                self.tick_operations(now)
                return
            if not self.alive():
                self.halt('机载控制进程已退出')
                return
            t = self.telemetry()
            if not self.robot.safe(t):
                self.halt('真实状态/IMU失效、关节故障或超速，已终止控制进程')
                return
            progress = self.backend.progress(self.read_log())
            if self.phase in ('loading', 'preparing', 'prepared') and progress.armed:
                self.halt('检测到未请求的策略接管')
                return
            if self.phase in ('loading', 'preparing'):
                if progress.prepared:
                    self.phase = 'prepared'
                elif progress.preparing:
                    self.phase = 'preparing'
            elif self.phase == 'starting_policy' and progress.active:
                self.phase = 'active'
                self.start_pending = False
            if self.phase in ('loading', 'preparing', 'starting_policy') and now > self.phase_deadline:
                self.halt('回正或策略接管超时')
                return
            if self.phase != 'active':
                return
            self.tick_pnlink(now)
            self.tick_pico(now)
            if self.phase != "active": return
            self.tick_mode(now)
            if self.phase != 'active': return
            self.tick_operations(now)
            if self.tracking_owned() and not self.task_active():
                self.last_packet=None;self.still_since=None
                return
            if self.input_mode != 'planner' or self.mode_pending:
                self.last_packet = None
                self.still_since = None
                return
            mode, movement, facing, speed = self.lease.sample(now)
            still = (t['upright'] and t['max_speed_rad_s'] < .6 and
                     not self.lease.move and not self.lease.turn)
            self.still_since = (now if self.still_since is None else self.still_since) if still else None
            self.last_packet = dict(mode=mode, movement_direction=movement,
                                    facing_direction=facing, speed=speed)

    def status(self):
        with self.lock:
            self.tick()
            t = self.telemetry()
            value = dict(sampled_at=self.clock(), state=self.phase, mode='onboard_real' if self.physical else 'onboard_isolated',
                        robot=self.robot.label, robot_id=self.robot.id, backend=self.backend.id,
                        backend_label=self.backend.label, capabilities=sorted(self.backend.capabilities),
                        motion_enabled=self.physical, pids=[self.process.pid] if self.alive() else [],
                        pose_ready=self.phase == 'prepared' and self.robot.safe(t),
                        prepare_available=self.phase in ('stopped', 'error') and not self.alive() and self.robot.safe(t),
                        control_transferred=self.phase == 'active',
                        prepare_seconds=self.backend.prepare_seconds, reference_planner_handoff_seconds=self.backend.handoff_seconds,
                        workflow_error=self.failure,
                        stationary=self.stationary(t),
                        default_speed=self.lease.default_speed, max_speed=self.backend.max_speed,
                        control_owner=self.lease.owner,
                        control_exclusive=True,
                        input_mode=self.input_mode, mode_switch_pending=self.mode_pending,
                        mode_switch_error=self.mode_error,
                        pose_adapter_ready=self.pose_reader is not None,
                        pose_input_ready=bool(self.pose_reader and self.pose_reader.snapshot()['ready']),
                        pose_input_detail=self.pose_reader.snapshot().get('reason','') if self.pose_reader else 'Pose 适配器未加载',
                        input_mode_switch_available=self.phase == 'active' and not self.mode_pending,
                        planner_control_allowed=self.input_mode == 'planner' and not self.mode_pending and not self.tracking_owned(),
                        telemetry={k: v for k, v in t.items() if k not in ('q', 'dq')},
                        epoch=self.epoch, ticket=self.ticket(), ticket_ttl_ms=250,
                        lease_remaining_ms=max(0, round((self.lease.deadline-self.clock())*1000)))
            value['input_source'] = {'planner': 'planner', 'pose': 'pico',
                                     'smpl_playback': 'file_smpl', 'joint_playback': 'file_joint'}.get(self.input_mode, self.input_mode)
            if self.pico_owned():value['input_source']='pico'
            if self.pnlink_owned():value['input_source']='pnlink'
            value['pnlink_session']=self.pnlink_session.status() if self.pnlink_session else None
            value['pnlink_command']=self.pnlink_command
            value['pnlink_command_pending']=bool(self.pnlink_session and self.pnlink_busy())
            value['tracking_inputs']={'pico':dict(ready=value['pose_input_ready']), 'pnlink':dict(ready=bool(self.pnlink_session and self.pnlink_session.ready()))}
            value['pico_session']=self.pico_session.status() if self.pico_session else None
            value['pico_mode']=self.pico_session.mode if self.pico_owned() else None
            value['pico_hands']=self.pico_hands_status()
            value['pnlink_hands']=self.pnlink_hands_status()
            value['pico_ready']=bool(self.pico_session and self.pico_session.ready())
            value['reference_representation'] = {'planner': 'planner', 'pose': 'smpl',
                                                  'smpl_playback': 'smpl', 'joint_playback': 'joint'}.get(self.input_mode)
            value['backend_feedback'] = self.policy_feedback[1] if self.policy_feedback else None
            value['playback'] = self.playback.snapshot() if self.playback else None
            value['playback_entry_checks'] = self.playback_entry_checks
            value['operator'] = self.operator_status()
            value['revision'] = self.revision
            value['motion_task'] = ({k:v for k,v in self.motion_task.items() if k not in ('deadline',)} if self.motion_task else None)
            value['conversion'] = dict(self.conversion) if self.conversion else None

            # Compatibility aliases for older HTTP clients.
            value['pico_pose_ready'] = value['pose_input_ready']
            value['pico_pose_detail'] = value['pose_input_detail']
            value.update(self.robot.feedback_status(self.latest,self.clock()))
            if not self.feedback_ready():
                for key in ('prepare_available','planner_control_allowed','input_mode_switch_available'):
                    value[key] = False
            value['planner_control_allowed'] &= 'planner' in self.backend.capabilities
            value['input_mode_switch_available'] &= 'pose' in self.backend.capabilities
            return value

    def pico_hands_status(self):
        if not self.pico_hand_link:return dict(enabled=False)
        try:
            status=self.pico_hand_link.status()
            if not isinstance(status,dict):raise ValueError('invalid optional hand status')
            if self.pico_hand_error:status=dict(status,error=self.pico_hand_error,output_disabled=True)
            return status
        except Exception:
            return dict(enabled=True,fresh=False,error='手部状态不可用，本体不受影响')

    def pnlink_hands_status(self):
        if not self.pnlink_hands:
            return dict(enabled=False)
        try:
            return dict(self.pnlink_hands.status(self.pico_hand_link), enabled=True)
        except Exception:
            return dict(enabled=True, error='手部状态不可用，本体不受影响')

    def stationary(self, t):
        return bool(self.input_mode == 'planner' and not self.mode_pending and self.phase == 'active' and self.robot.safe(t) and t.get('upright') and
                    t['max_speed_rad_s'] < .6 and
                    not self.lease.move and not self.lease.turn and self.still_since is not None and
                    self.clock()-self.still_since >= .5)

    def request(self, route, x):
        # Record discrete requests only. Never log tokens, tickets or joint data.
        discrete=bool(x.get('action') or x.get('stop'))
        info={'route':route,'action':x.get('action'),
              'stop':bool(x.get('stop')),'client':x.get('client')}
        if route == '/api/playback' and x.get('action') == 'play':
            checks = x.get('entry_checks', 'on')
            info['entry_checks'] = checks if checks in ('on', 'off') else 'invalid'
        info['monotonic'] = self.clock()
        if route == '/api/playback':
            checks=x.get('entry_checks', 'on')
            info['entry_checks'] = checks if checks in ('on','off') else 'invalid'
            for key in ('motion','request_id'):
                value=x.get(key)
                if isinstance(value,str):info[key]=value[:256]
        try:
            result = (self.load_playback(x) if route == '/api/playback' and x.get('action') == 'load'
                      else self._request(route,x))
        except (ValueError,RuntimeError) as exc:
            if discrete:logging.getLogger('control_audit').info(json.dumps(dict(info,result='rejected',reason=str(exc)),ensure_ascii=False))
            raise
        if discrete:
            playback=result.get('playback') or {}
            clip=playback.get('clip') or {}
            trace=dict(info,result='accepted',state=result.get('state'),input_source=result.get('input_source'),
                       reference_representation=result.get('reference_representation'),
                       motion_task=result.get('motion_task'),playback_state=playback.get('state'),
                       clip_name=clip.get('name'),clip_hashes=clip.get('hashes') or clip.get('cache_sha256'))
            logging.getLogger('control_audit').info(json.dumps(trace,ensure_ascii=False))
        return result

    def _request(self, route, x):
        with self.lock:
            if route not in ('/api/controller', '/api/control', '/api/playback', '/api/operator', '/api/motions', '/api/imports', '/api/inputs/pnlink'):
                raise ValueError('未知接口')
            if route == '/api/controller' and x.get('action') == 'estop':
                self.halt(open_hand=True)
                return self.status()
            stopping = ((route == '/api/control' and x.get('stop') is True) or
                        (route == '/api/playback' and x.get('action') == 'stop'))
            if not self.feedback_ready() and not stopping and route not in ('/api/operator', '/api/motions', '/api/imports', '/api/inputs/pnlink'):
                self.validate(x)
                raise ValueError('主备状态反馈未就绪或正在切换，暂不接受新的运动指令')
            self.tick()
            self.validate(x)
            for flag in ('stop',):
                if flag in x and type(x[flag]) is not bool:
                    raise ValueError('布尔控制参数无效')
            now = self.clock()
            if route == '/api/operator':
                self.operator_request(x)
                return self.status()
            if stopping:
                self.cancel_motion_task('操作者已停止', open_hand=True)
            else:
                self.require_operator(x, revision=route != '/api/control')
            if route in ('/api/motions', '/api/imports'):
                self.library_request(route, x)
                return self.status()
            if not stopping and route == '/api/controller' and x.get('action') in ('prepare_pose','start_policy'):
                if self.conversion and self.conversion['stage'] == 'running':
                    raise ValueError('请先完成或取消转换任务')
            hand_off = route == '/api/inputs/pnlink' and x.get('action') == 'configure_hands' and x.get('hand_mode') == 'off'
            if not stopping and not hand_off and route != '/api/playback' and self.task_active():
                raise ValueError('动作切换中，请先取消或停止')
            if not stopping and route != '/api/control':
                self.revision += 1
            if route == '/api/inputs/pnlink':
                self.pnlink_request(x)
            elif route == '/api/playback':
                self.playback_request(x, now)
            elif route == '/api/controller':
                action = x.get('action')
                if action=='select_pnlink':
                    self.start_pnlink_task(x.get('alignment_confirmed', False))
                elif action in ('pause_pnlink','resume_pnlink'):
                    if not self.pnlink_owned() or self.mode_pending:
                        raise ValueError('PN Link 未接管或正在切换')
                    if action=='pause_pnlink':self.pnlink_session.pause()
                    else:self.pnlink_session.resume()
                elif action=='select_pico' or (action=='select_pose' and self.pico_session is not None):
                    self.start_pico_task(x.get('pico_mode','pose'),x.get('alignment_confirmed',False))
                elif action in ('select_pose', 'select_planner'):
                    if action=='select_planner' and self.tracking_owned():
                        self.revoke_tracking('网页主动返回 Planner', open_hand=True)
                        self.begin_mode('planner','网页主动返回 Planner')
                        return self.status()
                    self.select_mode('pose' if action == 'select_pose' else 'planner', x)
                elif action == 'prepare_pose':
                    if self.phase not in ('stopped', 'error') or self.alive():
                        raise ValueError('请先急停再重新准备')
                    if not self.robot.safe(self.telemetry()):
                        raise ValueError('桥接反馈未就绪；未启动控制器')
                    # This guard must inspect live conflicts, not a cached preflight report.
                    self.guard()
                    # A slow guard must not turn stale feedback/tickets into permission.
                    self.validate_fresh(x)
                    if not self.robot.safe(self.telemetry()):
                        raise ValueError('检查期间反馈过期')
                    self.lease = PlannerLease(self.lease.default_speed, max_speed=self.backend.max_speed)
                    self.failure = ''
                    # Also retire legacy switch notices left by older runtimes.
                    self.mode_error = ''
                    self.mode_requested_at = 0.
                    self.still_since = None
                    self.phase = 'loading'
                    self.phase_deadline = self.clock()+self.backend.prepare_timeout
                    try:
                        self.process = self.launch()
                    except Exception as exc:
                        self.halt('启动失败：' + str(exc))
                        raise RuntimeError(self.failure) from exc
                elif action == 'start_policy':
                    if self.phase != 'prepared' or not self.robot.safe(self.telemetry()) or not self.alive():
                        raise ValueError('请先完成回正且保持有效反馈')
                    self.phase = 'starting_policy'
                    self.phase_deadline = now+self.backend.handoff_timeout
                    self.start_pending = True
                else:
                    raise ValueError('未知操作')
            else:
                self.control_request(x, now)
            return self.status()

    def begin_mode(self,mode,reason=''):
        self.lease.stop();self.last_packet=None
        # Native source switching reinitializes heading. Retaining the old
        # Planner integrator would reissue an old turn when idle resumes.
        self.lease.heading=0.;self.lease.last=None
        self.epoch=secrets.token_hex(16);self.sequence.clear()
        self.input_mode=mode;self.mode_pending=True;self.mode_requested_at=self.clock()
        self.mode_error=reason;self.still_since=None

    def select_mode(self,mode,x):
        if mode not in self.backend.capabilities:
            raise ValueError('控制器不支持此模式：'+mode)
        if self.phase!='active' or not self.alive() or not self.robot.safe(self.telemetry()):
            raise ValueError('请先启动控制器并保持反馈有效')
        if self.mode_pending:raise ValueError('模式正在切换，请等待控制器确认')
        if mode==self.input_mode:return
        if self.input_mode in ('joint_playback', 'smpl_playback'):
            if mode != 'planner':
                raise ValueError('请先停止文件播放并等待 Planner 确认，再切换追踪来源')
            self.playback.end()
        if mode=='pose':
            if not self.stationary(self.telemetry()):raise ValueError('请先停止行走并等待站稳')
            if not self.pose_reader or not self.pose_reader.snapshot()['ready']:raise ValueError('人体追踪姿态尚未就绪')
            if not self.policy_feedback or self.clock()-self.policy_feedback[0]>.5:
                raise ValueError('控制器 模式反馈未就绪')
            if not self.pose_reader.snapshot()['ready']:raise ValueError('检查期间人体追踪数据已过期')
        self.begin_mode(mode)

    def tick_mode(self,now):
        if self.input_mode in ('joint_playback', 'smpl_playback'):
            if not self.playback.healthy(now):
                self.playback_fallback('本地播放器产出超时')
            elif self.playback.state not in ('playing', 'paused'):
                self.playback_fallback(self.playback.error)
        if self.input_mode=='pose':
            if self.pico_owned():
                # PICO owns its bounded hold decision, including a reader loss
                # between the normal tick and this generic Pose watchdog.
                self.tick_pico(now)
                if self.input_mode!='pose':return
                ready=self.pico_owned()
            else:
                reader=self.active_pose_reader()
                ready=bool(reader and reader.snapshot()['ready'])
            if not ready:
                self.revoke_tracking('人体追踪断流')
                self.begin_mode('planner','人体追踪断流，已请求返回 Planner；需手动重新进入 Pose')
        if self.input_mode in ('pose', 'joint_playback', 'smpl_playback') and not self.mode_pending:
            if (not self.policy_feedback or now-self.policy_feedback[0]>.5 or
                self.policy_feedback[1].get('mode')!=('pose' if self.input_mode == 'smpl_playback' else self.input_mode) or self.policy_feedback[1].get('playing') is not True):
                if self.input_mode in ('joint_playback', 'smpl_playback'):
                    self.playback_fallback('控制器文件播放状态失效，返回 Planner')
                else:
                    self.revoke_tracking('控制器 Pose 状态失效')
                    self.begin_mode('planner','控制器 Pose 状态失效，返回 Planner')
        if not self.mode_pending:return
        if self.policy_feedback:
            at,value=self.policy_feedback
            expected=value.get('mode')==('pose' if self.input_mode == 'smpl_playback' else self.input_mode)
            if self.input_mode=='planner':
                pico_vr=(self.pico_owned() and self.motion_task and self.motion_task.get('kind')=='pico' and
                         self.motion_task['stage']=='starting' and self.pico_session.mode=='vr3pt')
                expected &= value.get('encoder_mode')==(1 if pico_vr else 0)
            if self.pico_owned() and self.motion_task and self.motion_task.get('kind')=='pico' and self.motion_task['stage']=='starting':
                expected &= value.get('encoder_mode')==(1 if self.pico_session.mode=='vr3pt' else 2 if self.pico_session.mode=='pose' else 0)
            if self.pnlink_owned():expected &= value.get('encoder_mode')==2
            if at>self.mode_requested_at+.1 and now-at<.25 and expected and value.get('playing') is True:
                self.mode_pending=False
                if self.input_mode in ('joint_playback', 'smpl_playback'):
                    self.playback.confirm(now)
                return
        if now-self.mode_requested_at>2:
            if self.input_mode in ('joint_playback', 'smpl_playback'):self.playback_fallback('文件播放接管未确认，返回 Planner')
            elif self.input_mode=='pose' or self.tracking_owned():
                self.revoke_tracking('PICO 接管未确认');self.begin_mode('planner','Pose 接管未确认，返回 Planner')
            else:self.halt('Planner 回退超时，控制器未确认模式')

    def enable_playback(self, root):
        if 'joint_playback' not in self.backend.capabilities:
            raise ValueError('后端不支持关节文件播放')
        from pathlib import Path
        self.motion_root = Path(root).resolve(strict=True)
        if not self.motion_root.is_dir():
            raise ValueError('动作根目录无效')
        self.playback = self.backend.file_player()
        if hasattr(self.backend, 'motion_library'):
            self.library = self.backend.motion_library(self.motion_root)

    def playback_fallback(self, error=''):
        state = 'error' if error else self.playback.state
        self.playback.end(state, error)
        self.begin_mode('planner', error or '文件播放结束，已请求零速 Planner')

    def playback_packet(self):
        """Only the owning transport calls this under the gateway lock."""
        if self.input_mode not in ('joint_playback', 'smpl_playback') or self.phase != 'active':
            return None
        try:
            packet = self.playback.packet(self.clock())
        except Exception as exc:
            self.playback_fallback('本地参考生成失败：'+str(exc))
            return None
        if packet is None:
            self.playback_fallback(self.playback.error)
        return packet

    def playback_request(self, x, now):
        if not self.playback:
            raise ValueError('文件播放未启用；需要配置 --motion-root')
        action = x.get('action')
        if action in ('switch', 'load_async'):
            self.start_motion_task(x, action == 'switch')
            return
        if action == 'cancel':
            self.cancel_motion_task('操作者已取消')
            return
        if self.task_active() and action not in ('stop',):
            raise ValueError('动作任务正在执行')
        if action == 'play':
            checks = x.get('entry_checks', 'on')
            if checks not in ('on', 'off'):
                raise ValueError('entry_checks 必须为 on 或 off')
            if not self.stationary(self.telemetry()) or not self.alive():
                raise ValueError('请人工准备、启动策略并等待 Planner 站稳')
            if not self.playback.clip:
                raise ValueError('请先加载动作')
            if not self.policy_feedback or now-self.policy_feedback[0] > .25 or self.policy_feedback[1].get('mode') != 'planner' or self.policy_feedback[1].get('playing') is not True:
                raise ValueError('缺少新鲜且明确的 Planner 模式反馈')
            if checks == 'on':
                self.backend.validate_first_frame(self.playback.clip, self.telemetry(), self.latest)
            self.playback.start(now)
            self.playback_entry_checks = checks
            self.begin_mode(self.playback.clip.mode)
        elif action in ('pause', 'resume'):
            if self.input_mode not in ('joint_playback', 'smpl_playback') or self.mode_pending:
                raise ValueError('文件播放未接管或正在切换')
            if action == 'pause':self.playback.pause()
            else:self.playback.resume(now)
        elif action == 'stop':
            if self.input_mode in ('joint_playback', 'smpl_playback'):
                self.playback.end()
                self.begin_mode('planner', '文件已停止，等待零速 Planner 确认')
        else:
            raise ValueError('未知播放操作')

    def load_playback(self, x):
        # File parsing is outside the control mutex. Preparation racing with
        # an import invalidates its result rather than attaching it to a run.
        with self.lock:
            self.validate(x)
            self.tick()
            self.require_operator(x)
            if self.task_active():
                raise ValueError('已有动作任务，请等待或取消')
            if self.phase == 'active':
                self.start_motion_task(x, False)
                return self.status()
            if not self.playback:
                raise ValueError('文件播放未启用；需要配置 --motion-root')
            if self.phase not in ('stopped', 'error') or self.alive() or self.mode_pending:
                raise ValueError('请在控制器未接管时加载动作')
            self.revision += 1
            revision = self.revision
            epoch = self.epoch
            from pathlib import Path
            name = x.get('motion')
            if not isinstance(name, str) or not name or Path(name).is_absolute():
                raise ValueError('motion 必须为动作根目录内的相对路径')
            path = (self.motion_root / name).resolve()
            if path == self.motion_root or self.motion_root not in path.parents:
                raise ValueError('动作路径越界')
        clip = self.backend.load_clip(path, fps=x.get('fps'), joint_order=x.get('joint_order'),
                                      root_body_index=x.get('root_body_index', 0))
        with self.lock:
            if self.epoch != epoch or self.revision != revision or self.phase not in ('stopped', 'error') or self.alive():
                raise ValueError('加载期间控制状态已改变；未替换动作')
            self.playback.load(clip)
            self.playback_entry_checks = None
            return self.status()

    def validate_fresh(self, x):
        # validate() already consumed the sequence; only recheck its signed timestamp.
        if self.clock()-float(x['ticket'].split(':')[0]) > .25:
            raise ValueError('冲突检查期间请求已过期，请重试')

    def control_request(self, x, now):
        if x.get('stop') is True:
            self.lease.accept(x, now)
            pico=self.tracking_owned();self.revoke_tracking('操作者已停止遥操', open_hand=True)
            if self.phase == 'active' and (self.input_mode != 'planner' or pico):
                if self.playback and self.input_mode in ('joint_playback', 'smpl_playback'):self.playback.end()
                self.begin_mode('planner', '已请求停止参考输入，返回 Planner 待命')
            return
        if 'planner' not in self.backend.capabilities:
            raise ValueError('控制器不支持 Planner')
        if self.input_mode != 'planner' or self.mode_pending or self.tracking_owned():
            raise ValueError('参考输入接管或切换中，Planner 指令已拒绝')
        if self.phase != 'active' or not self.alive() or not self.robot.safe(self.telemetry()):
            raise ValueError('请先准备姿态并启动 控制器，且保持有效反馈')
        self.lease.accept(x, now)
