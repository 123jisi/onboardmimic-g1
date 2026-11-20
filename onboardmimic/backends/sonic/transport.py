import json, logging, threading, time, sys
from onboardmimic.core.runtime_timing import wait_event

def runtime_event(event, **fields):
    logging.getLogger("runtime_audit").info(json.dumps(dict(event=event, **fields)))

class Transport:
    def __init__(self, gateway, state_port=16660, planner_port=16686):
        self.gateway = gateway
        self.state_port, self.planner_port = state_port, planner_port
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.last_tick = time.monotonic()
        self.error = None
        self.stage = 'initializing'
        self.shutdown_reason = None
        self.last_pose_key = None
        self.hand_blocked_session = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def messages(self):
        messages=self.body_messages()
        link=self.gateway.pico_hand_link
        if link:
            # Optional hand failures must never escape into the body transport
            # watchdog. Keep the body reference already produced above.
            candidate=self.gateway.pnlink_hands
            pn=candidate if candidate and (self.gateway.pnlink_owned() or candidate.finishing()) else None
            session=((pn.finish_task['session']+'-finish' if pn.finishing() else pn.session) if pn else getattr(self.gateway.pico_session,'hand_session',None)) or 'inactive'
            if self.hand_blocked_session!=session:
                self.hand_blocked_session=None
                self.gateway.pico_hand_error=''
            if self.hand_blocked_session is None:
                try:
                    hand=pn.message(self.gateway,link) if pn else link.message(self.gateway)
                    if hand is not None:
                        if not isinstance(hand,bytes) or not hand.startswith(b'pico_hands'):
                            raise ValueError('invalid optional hand packet')
                        messages.append(hand)
                except Exception as exc:
                    self.hand_blocked_session=session
                    if pn:
                        pn.fail('手部输出异常，已停用手部；本体继续运行，重新进入可重试')
                    else:
                        self.gateway.pico_hand_error='手部输出异常，已停用手部；本体继续运行，重新进入 PICO 可重试'
                    runtime_event('pico_hand_output_disabled',error=type(exc).__name__)
        return messages

    def body_messages(self):
        """Called under gateway.lock; this is the sole control/reference mux."""
        from .wire import build_command_message, build_planner_message
        g = self.gateway
        idle = lambda: build_planner_message(0, [0,0,0], [1,0,0], speed=0)
        if g.start_pending:
            self.last_pose_key = None
            return [idle(), build_command_message(True, False, True)]
        if g.phase != 'active':
            self.last_pose_key = None
            return []
        if g.pnlink_owned():
            packet=g.pnlink_reference(g.clock())
            if packet is None:return [idle(),build_command_message(True,False,True)]
            return [packet]+([build_command_message(True,False,False)] if g.mode_pending else [])
        if g.pico_owned():
            reference=g.pico_reference(g.clock())
            # A failed reference may have revoked the session; publish only the
            # explicit fallback, never the previous command or cached pose.
            if reference is None:
                return [idle(),build_command_message(True,False,True)]
            planner=isinstance(reference,dict)
            if planner:
                packet=reference
                message=build_planner_message(packet['mode'],packet['movement_direction'],packet['facing_direction'],speed=packet['speed'],
                    **{k:packet[k] for k in ('height','upper_body_position','upper_body_velocity','vr_3pt_position','vr_3pt_orientation') if k in packet})
            else:message=reference
            return [message]+([build_command_message(True,False,planner)] if g.mode_pending else [])
        if g.input_mode != 'pose':
            self.last_pose_key = None
        if g.input_mode in ('joint_playback', 'smpl_playback'):
            packet = g.playback_packet()
            # Finishing can change the selected mode during packet production.
            if g.input_mode == 'planner':
                return [idle(), build_command_message(True, False, True)]
            messages = [build_command_message(True, False, False)] if g.mode_pending else []
            return messages + ([packet] if packet else [])
        if g.input_mode == 'pose' or g.mode_pending:
            is_planner = g.input_mode == 'planner'
            messages = [build_command_message(True, False, is_planner)] if g.mode_pending else []
            if is_planner:
                return [idle()] + messages
            snap = g.pose_reader.snapshot()
            key = (g.epoch, snap.get('packet'))
            if snap['ready'] and key != self.last_pose_key:
                messages.append(snap['packet'])
                self.last_pose_key = key
            return messages
        packet = g.last_packet
        if packet:
            return [build_planner_message(
                packet['mode'], packet['movement_direction'], packet['facing_direction'], speed=packet['speed'],
                **{k:packet[k] for k in ('height','upper_body_position','upper_body_velocity') if k in packet})]
        return []

    def run(self):
        import msgpack
        import zmq
        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        pub = ctx.socket(zmq.PUB)
        feedback = ctx.socket(zmq.SUB)
        exception_info = None
        try:
            sub.setsockopt(zmq.SUBSCRIBE, b'rt/')
            sub.setsockopt(zmq.RCVHWM, 100)
            sub.connect('tcp://127.0.0.1:%d' % self.state_port)
            pub.setsockopt(zmq.SNDHWM, 5)
            pub.bind('tcp://127.0.0.1:%d' % self.planner_port)
            feedback.setsockopt(zmq.SUBSCRIBE, b'g1_debug')
            feedback.setsockopt(zmq.CONFLATE, 1)
            feedback.connect('tcp://127.0.0.1:%d' % (self.planner_port+1))
            self.ready.set()
            while not self.stop.is_set():
                begin = time.monotonic()
                self.stage = 'state_poll'
                if sub.poll(5):
                    self.stage = 'state_receive'
                    for _ in range(100):
                        try:
                            topic, payload = sub.recv_multipart(zmq.NOBLOCK)
                            value = msgpack.unpackb(payload, raw=False)
                            self.gateway.ingest(topic.decode(), value)
                        except zmq.Again:
                            break
                        except (ValueError, TypeError, UnicodeDecodeError):
                            # Preserve last valid timestamp: malformed traffic cannot renew freshness.
                            continue
                self.stage = 'policy_feedback'
                if feedback.poll(0):
                    try:
                        raw = feedback.recv(zmq.NOBLOCK)
                        self.gateway.ingest('g1_debug', msgpack.unpackb(raw[len(b'g1_debug'):], raw=False))
                    except (zmq.Again, ValueError, TypeError): pass
                self.stage = 'gateway_tick'
                with self.gateway.lock:
                    self.gateway.tick()
                    for message in self.messages():
                        pub.send(message)
                self.last_tick = time.monotonic()
                self.stage = 'cadence_wait'
                wait_event(self.stop, max(0, .02-(self.last_tick-begin)))
        except BaseException as exc:
            self.error = type(exc).__name__+': '+str(exc)
            self.shutdown_reason = 'transport_exception'
            exception_info = sys.exc_info()
        finally:
            with self.gateway.lock:
                self.gateway.halt('机载通信循环已停止' + (': '+self.error if self.error else ''))
            if exception_info:
                logging.getLogger('runtime_audit').error('Transport failed at %s',self.stage,exc_info=exception_info)
            runtime_event('transport_exit',stage=self.stage,error=self.error,stop_requested=self.stop.is_set())
            sub.close(0); pub.close(0); feedback.close(0); ctx.term()
            self.ready.set()
