"""SONIC controller integration; no hardware I/O during discovery."""
import json
import os
from pathlib import Path
from onboardmimic.core.contracts import S0Backend, LaunchSpec, BackendProgress

DEPLOY = Path(__file__).resolve().parent/'deploy'
RESOURCES = DEPLOY/'onboard'


def controller_args(root, interface, robot=None):
    from onboardmimic.core.paths import ROOT
    # root is the backend resource directory, retained for diagnostic callers.
    deploy = Path(root).parent
    return [str(deploy/'target/release/g1_deploy_onnx_ref'), interface,
        'policy/release/model_decoder.onnx', 'reference/example/',
        '--obs-config','policy/release/observation_config.yaml',
        '--encoder-file','policy/release/model_encoder.onnx',
        '--planner-file','planner/target_vel/V2/planner_orin_compatible.onnx',
        '--input-type','zmq_manager','--output-type','zmq','--hand-type','none',
        '--zmq-host','127.0.0.1','--zmq-port','16686','--zmq-out-port','16687',
        '--zmq-robot-ip','127.0.0.1','--zmq-robot-state-port',str(robot.state_port if robot else 16660),
        '--zmq-robot-cmd-port',str(robot.command_port if robot else 16661),
        '--zmq-robot-lowcmd-udp-port',str(robot.command_udp_port if robot else 16663)]


class SonicBackend(S0Backend):
    id = 'sonic'
    label = 'SONIC'
    compatible_robots = frozenset(('unitree_g1',))
    capabilities = frozenset(('planner','pose','joint_playback','smpl_playback'))
    supported_inputs = frozenset(('pico','pnlink'))

    def installer(self, robot):
        if robot.id not in self.compatible_robots:
            return super().installer(robot)
        from .setup import main
        return main

    def baseline(self, robot):
        return json.loads((RESOURCES/'baseline_q.json').read_text())

    def launch_spec(self, robot):
        interface, hardware_env = robot.controller_environment()
        env = dict(os.environ, SONIC_SIM_GESTURES='0', HAND_TYPE='none', **hardware_env)
        env.pop('SONIC_LOWCMD_ZMQ_FALLBACK',None)
        return LaunchSpec(controller_args(RESOURCES,interface,robot),DEPLOY,env)

    def progress(self, text):
        return BackendProgress(preparing='[LowCmd TX] transport=' in text,
            prepared='Init Done' in text, active='Bumpless policy handoff complete' in text,
            armed='CONTROL armed' in text)

    def decode_status(self, topic, value):
        if topic != 'g1_debug' or not isinstance(value,dict):
            return None
        encoder = value.get('encoder_mode')
        name = value.get('motion_name')
        encoder = encoder if type(encoder) is int else None
        name = name if isinstance(name, str) and len(name) <= 128 else None
        # Encoder 0 is used by both Planner and joint references. Only the
        # native motion identity can distinguish them; missing identity fails
        # closed instead of acknowledging an unrelated mode.
        mode = ('planner' if name == 'planner_motion' and encoder in (0, 1) else
                'joint_playback' if name == 'streamed' and encoder == 0 else
                'pose' if name == 'streamed' and encoder == 2 else None)
        return dict(mode=mode, encoder_mode=encoder, motion_name=name,
                    playing=value.get('play') is True)

    def transport(self, gateway, robot):
        from .transport import Transport
        return Transport(gateway,state_port=robot.state_port)

    def pose_reader(self, input_id, path):
        if input_id == 'pnlink':
            from .pnlink.reader import PoseReader
            return PoseReader(path)
        if input_id != 'pico':
            return super().pose_reader(input_id,path)
        from .pico.pose_reader import PoseReader
        return PoseReader(path)

    def pico_session(self,reader):
        from .pico.session import PicoSession
        return PicoSession(reader,self.max_speed)

    def pnlink_session(self, reader):
        from .pnlink.session import PNLinkSession
        return PNLinkSession(reader)

    def pnlink_hands(self, reader, path):
        from .pnlink.hands import PNLinkHands
        return PNLinkHands(reader,path)

    def pnlink_command(self, path, action, receiver_id, request_id):
        from onboardmimic.inputs.pnlink.state import command
        return command(path, action, receiver_id, request_id)

    def pnlink_service(self, action):
        from onboardmimic.inputs.pnlink.service import change
        return change(action)

    def pico_hand_link(self,robot,path):
        if robot.id!='unitree_g1':
            return super().pico_hand_link(robot,path)
        from .pico.hands import PicoHandLink
        return PicoHandLink(path)

    def file_player(self):
        from .csv_player import CSVPlayer
        return CSVPlayer()

    def load_clip(self, path, **options):
        from .csv_player import CSVClip
        if Path(path).suffix == ".npz":
            from .smpl_player import SMPLClip
            return SMPLClip(path)
        return CSVClip(path, **options)

    def motion_library(self, root):
        from .motion_library import MotionLibrary
        return MotionLibrary(root)

    def prepare_clip(self, root, spec, cancelled):
        from .prepared_clip import prepare_clip
        return prepare_clip(root, spec, cancelled)

    def validate_static_first_frame(self, clip):
        # This can fail before interrupting the currently playing reference.
        if max(abs(float(v)) for v in clip.dq[0]) > .6:
            raise ValueError('首帧参考速度超过 0.6 rad/s；请提供静止起始段')

    def validate_first_frame(self, clip, telemetry, latest):
        # The on-site operator explicitly allows a first-frame joint-position
        # discontinuity. Position alignment is no longer an admission gate.
        if max(abs(float(v)) for v in clip.dq[0]) > .6:
            raise ValueError('首帧参考速度超过 0.6 rad/s；请提供静止起始段')
        imu = latest['rt/secondary_imu'][1]['quaternion']
        if clip.tilt_error(imu) > .15:
            raise ValueError('首帧根部倾斜与实测姿态偏差超过 0.15 rad；未接管')
