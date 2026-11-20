"""No-hardware admission, failure and real loopback checks for local CSV."""
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
import subprocess
import sys
import re
from unittest.mock import Mock, patch

import numpy as np

from onboardmimic.backends.sonic.csv_player import CSVClip, REFERENCE_TO_HARDWARE
from onboardmimic.backends.sonic.transport import Transport
from onboardmimic.backends.sonic.adapter import SonicBackend
from onboardmimic.backends.sonic.wire import HEADER_SIZE
from tests.fixtures import NativeLifecycle
from tests.runtime import test_native_lifecycle as native_fixtures
Process = native_fixtures.Process


def write_clip(path, frames=40):
    path.mkdir()
    q = np.repeat(np.arange(frames, dtype=float)[:, None] * .001, 29, axis=1)
    dq = np.full_like(q, .05)
    dq[0] = 0
    for name, values in [('joint_pos', q), ('joint_vel', dq),
                         ('body_quat', np.tile([1., 0, 0, 0], (frames, 1)))]:
        np.savetxt(path / (name+'.csv'), values, delimiter=',',
                   header=','.join('v%d'%i for i in range(values.shape[1])), comments='')


def unpack(raw, topic=b'pose'):
    header = json.loads(raw[len(topic):len(topic)+HEADER_SIZE].rstrip(b'\0'))
    data = memoryview(raw)[len(topic)+HEADER_SIZE:]
    fields = {}
    for field in header['fields']:
        dtype = {'f32':'<f4', 'f64':'<f8', 'i64':'<i8', 'i32':'<i4', 'u8':'u1'}[field['dtype']]
        count = int(np.prod(field['shape']))
        array = np.frombuffer(data, dtype=dtype, count=count).reshape(field['shape'])
        fields[field['name']] = array
        data = data[array.nbytes:]
    return header, fields


class LocalPlaybackTest(unittest.TestCase):
    refresh = native_fixtures.NativeTest.refresh
    packet = native_fixtures.NativeTest.packet
    command = native_fixtures.NativeTest.command
    activate = native_fixtures.NativeTest.activate

    def setUp(self):
        native_fixtures.NativeTest.setUp(self)
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        write_clip(self.root/'short')
        self.g.enable_playback(self.root)
        self.call('load', motion='short', fps=50, joint_order='sonic_reference')
        self.transport = Transport(self.g)

    def call(self, action, **kw):
        return self.g.request('/api/playback', self.packet(action=action, **kw))

    def feedback(self, mode=None, **kw):
        mode = mode or self.g.input_mode
        self.g.ingest('g1_debug', dict(encoder_mode=2 if mode=='pose' else 0,
                                     motion_name='planner_motion' if mode=='planner' else 'streamed',
                                     play=True, **kw))

    def begin(self):
        self.activate()
        self.now += .6
        self.refresh()
        self.g.tick()
        self.feedback('planner')
        self.call('play')
        self.transport.messages()

    def step(self, count=1, ack=True):
        result = []
        for _ in range(count):
            self.now += .02
            self.refresh()
            if ack:self.feedback()
            self.g.tick()
            result = self.transport.messages()
        return result

    def test_load_and_restart_never_start_or_select_source(self):
        self.assertEqual(self.launches, 0)
        self.assertEqual(self.g.input_mode, 'planner')
        self.assertEqual(self.g.playback.state, 'loaded')
        with self.assertRaises(ValueError):self.call('play')
        other = NativeLifecycle([0.]*29, lambda:Process(), lambda:'', lambda:None)
        other.enable_playback(self.root)
        self.assertEqual(other.playback.state, 'unloaded')

    def test_mode_zero_needs_native_identity(self):
        backend = SonicBackend()
        for name, mode in [(None,None), ('other',None), ('streamed','joint_playback'),
                           ('planner_motion','planner')]:
            self.assertEqual(backend.decode_status('g1_debug',dict(encoder_mode=0, motion_name=name, play=True))['mode'],mode)
        for encoder,name in [(True,'planner_motion'), (float('nan'),b'streamed'), ([0],{})]:
            status=backend.decode_status('g1_debug',dict(encoder_mode=encoder,motion_name=name,play=True))
            self.assertIsNone(status['mode'])
            json.dumps(status,allow_nan=False)

    def test_joint_confirmation_then_timeline_and_single_mux(self):
        self.begin()
        self.assertTrue(self.g.mode_pending)
        self.step(4)
        self.assertEqual(self.g.playback.position, 0)
        messages = self.step(4)
        self.assertFalse(self.g.mode_pending)
        self.assertGreater(self.g.playback.position, 0)
        self.assertEqual(len(messages), 1)
        header, data = unpack(messages[0])
        self.assertEqual(header['v'], 1)
        self.assertEqual(data['joint_pos'].shape, (64,29))
        self.assertEqual(data['body_quat'].shape, (64,1,4))
        self.assertAlmostEqual(data['timestamp_monotonic'][0], self.now)
        frame = int(self.g.playback.position*50+1e-7)
        np.testing.assert_allclose(data['joint_pos'][0], self.g.playback.clip.q[frame])
        status = self.g.status()
        self.assertEqual(status['input_source'], 'file_joint')
        self.assertEqual(status['reference_representation'], 'joint')
        with self.assertRaises(ValueError):self.g.request('/api/control',self.packet(move='forward'))

    def test_pause_holds_reference_but_advances_wire_time_and_resume_no_wall_gap(self):
        self.begin();self.step(10);self.call('pause')
        position = self.g.playback.position
        first = unpack(self.step()[0])[1]
        last = unpack(self.step(40)[0])[1]
        self.assertEqual(self.g.playback.position, position)
        np.testing.assert_array_equal(first['joint_pos'], last['joint_pos'])
        self.assertTrue((last['joint_vel']==0).all())
        self.assertGreater(last['frame_index'][0], first['frame_index'][0])
        self.assertGreater(last['timestamp_monotonic'][0], first['timestamp_monotonic'][0])
        self.call('resume');self.step()
        self.assertAlmostEqual(self.g.playback.position, position+.02)
        self.assertEqual(self.g.input_mode, 'joint_playback')

    def test_finish_explicitly_returns_idle_planner_and_requires_identity_ack(self):
        self.begin()
        for _ in range(100):
            self.step()
            if self.g.input_mode=='planner':break
        self.assertEqual(self.g.playback.state, 'finished')
        self.assertTrue(self.g.mode_pending)
        self.assertIsNone(self.g.playback_packet())
        for _ in range(8):
            self.now+=.02;self.refresh()
            self.g.ingest('g1_debug',dict(encoder_mode=0,motion_name='streamed',play=True))
            self.g.tick()
        self.assertTrue(self.g.mode_pending)
        self.feedback('planner');self.g.tick()
        self.assertFalse(self.g.mode_pending)
        self.assertEqual(self.g.last_packet['speed'],0)
        self.assertEqual(self.g.playback.state,'finished')

    def test_stop_and_replay_clear_indices_and_epoch(self):
        self.begin();self.step(10)
        self.g.lease.heading=1.2
        epoch=self.g.epoch
        old = self.packet(action='resume')
        self.call('stop')
        self.assertNotEqual(self.g.epoch,epoch)
        self.assertEqual(self.g.playback.state,'loaded')
        self.assertEqual(self.g.playback.stream_index,0)
        self.assertEqual(self.g.lease.heading,0.)
        self.assertIsNone(self.g.playback_packet())
        with self.assertRaises(ValueError):self.g.request('/api/playback',old)
        self.step(40)
        self.call('play')
        data=unpack(self.transport.messages()[-1])[1]
        self.assertEqual(data['frame_index'][0],0)
        np.testing.assert_array_equal(data['joint_pos'],np.repeat(self.g.playback.clip.q[:1],64,axis=0))

    def test_control_stop_works_during_pending_or_paused(self):
        for paused in (False,True):
            if paused:
                self.step(40);self.call('play');self.transport.messages();self.step(10);self.call('pause')
            else:self.begin()
            self.g.request('/api/control',self.packet(stop=True))
            self.assertEqual(self.g.input_mode,'planner')
            self.assertTrue(self.g.mode_pending)
            self.assertEqual(self.g.playback.state,'loaded')

    def test_producer_stall_even_paused_requests_fallback(self):
        self.begin();self.step(10);self.call('pause')
        self.now+=.151;self.refresh();self.feedback()
        self.g.tick()
        self.assertEqual(self.g.input_mode,'planner')
        self.assertEqual(self.g.playback.state,'error')
        self.assertIn('超时',self.g.playback.error)

    def test_wrong_mode_ack_and_planner_timeout_kill_owned_only(self):
        self.begin()
        for _ in range(110):self.step(ack=False)
        self.assertEqual(self.g.input_mode,'planner')
        self.assertEqual(self.g.playback.state,'error')
        for _ in range(110):self.step(ack=False)
        self.assertEqual(self.g.phase,'error')
        self.assertEqual(self.proc.kills,1)
        self.assertEqual(self.launches,1)

    def test_stale_or_wrong_active_feedback_falls_back(self):
        for wrong in (False,True):
            if wrong:self.setUp()
            self.begin();self.step(10)
            if wrong:
                self.feedback('planner');self.g.tick()
            else:
                for _ in range(27):self.step(ack=False)
            self.assertEqual(self.g.input_mode,'planner')
            self.assertEqual(self.g.playback.state,'error')

    def test_admission_requires_fresh_feedback_quiet_velocity_and_tilt(self):
        self.activate();self.now+=.6;self.refresh();self.g.tick()
        with self.assertRaises(ValueError):self.call('play')
        self.feedback('planner')
        self.g.playback.clip.dq.flags.writeable=True
        self.g.playback.clip.dq[0,0]=.601
        with self.assertRaisesRegex(ValueError,'速度'):self.call('play')
        self.g.playback.clip.dq[0,0]=0
        self.g.playback.clip.quat.flags.writeable=True
        self.g.playback.clip.quat[0]=[np.cos(.2),np.sin(.2),0,0]
        with self.assertRaisesRegex(ValueError,'倾斜'):self.call('play')
        self.assertEqual(self.g.input_mode,'planner')

    def test_first_frame_position_jump_is_allowed_and_sent_without_alignment(self):
        clip=self.g.playback.clip
        clip.q.flags.writeable=True
        clip.q[0]=1.
        self.assertGreater(clip.alignment_error([0.]*29),.15)
        self.begin()
        self.assertEqual(self.g.input_mode,'joint_playback')
        self.assertTrue(self.g.mode_pending)
        data=unpack(self.transport.messages()[-1])[1]
        np.testing.assert_array_equal(data['joint_pos'],np.ones((64,29),dtype=np.float32))
        self.assertEqual(self.g.playback.position,0.)

    def test_entry_checks_off_allows_velocity_and_tilt_but_still_requires_feedback(self):
        self.activate();self.now+=.6;self.refresh();self.g.tick()
        clip=self.g.playback.clip
        clip.dq.flags.writeable=True;clip.dq[0,0]=2.
        clip.quat.flags.writeable=True;clip.quat[0]=[np.cos(.2),np.sin(.2),0,0]
        with self.assertRaisesRegex(ValueError,'反馈'):
            self.call('play',entry_checks='off')
        self.feedback('planner')
        with self.assertRaisesRegex(ValueError,'速度'):
            self.call('play')
        with self.assertLogs('control_audit',level='INFO') as logs:
            result=self.call('play',entry_checks='off')
        self.assertEqual(result['playback_entry_checks'],'off')
        self.assertTrue(any('"entry_checks": "off"' in line for line in logs.output))
        self.assertTrue(self.g.mode_pending)
        self.step(8)
        self.assertFalse(self.g.mode_pending)
        self.feedback('planner');self.g.tick()
        self.assertEqual(self.g.input_mode,'planner')
        self.assertEqual(self.g.playback.state,'error')

    def test_entry_switch_is_per_play_and_invalid_values_never_change_source(self):
        self.activate();self.now+=.6;self.refresh();self.g.tick();self.feedback('planner')
        for value in (None,True,False,0,'disabled',[]):
            with self.subTest(value=value),self.assertRaises(ValueError):
                self.call('play',entry_checks=value)
            self.assertEqual(self.g.input_mode,'planner')
            self.assertIsNone(self.g.playback_entry_checks)
        self.call('play',entry_checks='off');self.transport.messages();self.step(8)
        self.call('stop');self.step(40)
        clip=self.g.playback.clip;clip.dq.flags.writeable=True;clip.dq[0,0]=2.
        with self.assertRaisesRegex(ValueError,'速度'):
            self.call('play')
        self.assertEqual(self.g.input_mode,'planner')

    def test_entry_checks_off_does_not_skip_manual_start_or_feedback_watchdogs(self):
        with self.assertRaises(ValueError):self.call('play',entry_checks='off')
        self.activate();self.now+=.6;self.refresh();self.g.tick();self.feedback('planner')
        self.call('play',entry_checks='off');self.transport.messages()
        self.step(110,ack=False)
        self.assertEqual(self.g.input_mode,'planner')
        self.assertEqual(self.g.playback.state,'error')
        self.step(110,ack=False)
        self.assertEqual(self.g.phase,'error')
        self.assertEqual(self.proc.kills,1)

    def test_actual_native_joint_gather_is_used(self):
        clip=self.g.playback.clip
        clip.q.flags.writeable=True
        hardware=np.arange(29,dtype=float)*.03
        source=(Path(__file__).resolve().parents[2]/'onboardmimic/backends/sonic/deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp').read_text()
        values=re.search(r'mujoco_to_isaaclab\s*=\s*\{([^}]+)',source).group(1)
        native_mapping=[int(value) for value in values.split(',') if value.strip()]
        self.assertEqual(tuple(native_mapping),REFERENCE_TO_HARDWARE)
        clip.q[0]=hardware[native_mapping]
        self.assertLess(clip.alignment_error(hardware),1e-7)
        self.assertGreater(clip.alignment_error(hardware[::-1]),.15)

    def test_load_rejected_while_active_and_path_escape_or_invalid_data(self):
        with self.assertRaises(ValueError):self.call('load',motion='../outside',fps=50,joint_order='sonic_reference')
        with self.assertRaises(ValueError):self.call('load',motion='short',fps=30,joint_order='sonic_reference')
        with self.assertRaises(ValueError):self.call('load',motion='short',fps=50,joint_order='mujoco')
        (self.root/'escape').symlink_to(self.root.parent)
        with self.assertRaises(ValueError):self.call('load',motion='escape',fps=50,joint_order='sonic_reference')
        write_clip(self.root/'bad')
        (self.root/'bad/joint_pos.csv').write_text('q\nnan\n0\n')
        with self.assertRaises(ValueError):self.call('load',motion='bad',fps=50,joint_order='sonic_reference')
        self.begin()
        clip=self.g.playback.clip
        with self.assertRaises(ValueError):self.call('load',motion='short',fps=50,joint_order='sonic_reference')
        self.assertIs(self.g.playback.clip,clip)

    def test_import_does_not_hold_control_mutex_and_estop_invalidates_result(self):
        original=CSVClip
        def loader(*a,**kw):
            acquired=[]
            def stop():
                with self.g.lock:
                    acquired.append(True);self.g.halt()
            worker=threading.Thread(target=stop);worker.start();worker.join(1)
            self.assertEqual(acquired,[True])
            return original(*a,**kw)
        with patch('onboardmimic.backends.sonic.csv_player.CSVClip',side_effect=loader):
            with self.assertRaisesRegex(ValueError,'加载期间'):
                self.call('load',motion='short',fps=50,joint_order='sonic_reference')

    def test_file_pico_roundtrip_uses_planner_barrier_and_fresh_packets(self):
        self.begin();self.step(10)
        self.g.pose_reader=Mock()
        self.g.pose_reader.snapshot.return_value=dict(ready=True,packet=b'pose-current')
        with self.assertRaises(ValueError):self.command('select_pose')
        self.call('stop');self.step(40);self.command('select_pose');self.step(8)
        self.assertFalse(self.g.mode_pending)
        self.assertEqual(self.g.input_mode,'pose')
        with self.assertRaises(ValueError):self.call('play')
        self.command('select_planner');self.step(40)
        self.call('play');messages=self.transport.messages()
        self.assertTrue(all(m != b'pose-current' for m in messages))
        self.assertEqual(unpack(messages[-1])[1]['frame_index'][0],0)

    def test_generator_exception_uses_explicit_fallback(self):
        self.begin();self.step(8)
        with patch.object(self.g.playback,'packet',side_effect=ValueError('broken frame')):
            messages=self.transport.messages()
        self.assertEqual(self.g.input_mode,'planner')
        self.assertEqual(self.g.playback.state,'error')
        self.assertEqual([m.startswith(b'pose') for m in messages],[False,False])

    def test_malformed_shapes_quaternions_and_file_symlinks_are_rejected(self):
        for case in ('width','length','quaternion','file_link','missing_header'):
            with self.subTest(case=case):
                path=self.root/case;write_clip(path)
                if case=='width':
                    np.savetxt(path/'joint_pos.csv',np.zeros((40,28)),delimiter=',',header=','.join('q%d'%i for i in range(28)),comments='')
                elif case=='length':
                    np.savetxt(path/'joint_vel.csv',np.zeros((39,29)),delimiter=',',header=','.join('dq%d'%i for i in range(29)),comments='')
                elif case=='quaternion':
                    np.savetxt(path/'body_quat.csv',np.zeros((40,4)),delimiter=',',header='w,x,y,z',comments='')
                elif case=='file_link':
                    (path/'joint_pos.csv').unlink()
                    (path/'joint_pos.csv').symlink_to(self.root/'short/joint_pos.csv')
                else:
                    p=path/'joint_pos.csv';p.write_text('\n'.join(p.read_text().splitlines()[1:])+'\n')
                with self.assertRaises(ValueError):
                    CSVClip(path,fps=50,joint_order='sonic_reference')

    def test_stop_remains_available_when_continuity_is_not_ready(self):
        self.begin();self.step(8)
        self.g.require_continuity=True
        self.call('stop')
        self.assertEqual(self.g.input_mode,'planner')
        self.assertTrue(self.g.mode_pending)


class LoopbackPlaybackTest(unittest.TestCase):
    def test_local_cli_uses_authenticated_api_without_starting_anything(self):
        from onboardmimic.core.gateway import make_server
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);write_clip(root/'short')
            g=NativeLifecycle([0.]*29,lambda:Process(),lambda:'',lambda:None)
            g.enable_playback(root)
            token=root/'http.token';token.write_text('test'*16);token.chmod(0o600)
            server=make_server(g,token.read_text(),0)
            thread=threading.Thread(target=server.serve_forever,kwargs={'poll_interval':.01})
            thread.start()
            try:
                script=Path(__file__).resolve().parents[2]/'scripts/playback.py'
                result=subprocess.run([sys.executable,str(script),'load','short','--fps','50',
                                       '--joint-order','sonic_reference','--token-file',str(token),
                                       '--url','http://127.0.0.1:%d'%server.server_port],
                                      cwd=folder,capture_output=True,text=True,timeout=5)
                self.assertEqual(result.returncode,0,result.stderr)
                status=json.loads(result.stdout)
                self.assertEqual(status['state'],'stopped')
                self.assertEqual(status['pids'],[])
                self.assertEqual(status['workflow_error'],'')
                self.assertEqual(status['playback']['state'],'loaded')
                self.assertIsNone(g.process)
                self.assertNotIn(token.read_text(),result.stdout)
                # A deterministic fake controller tests the actual CLI/HTTP
                # option without a wall-clock race or any hardware process.
                g.clock=lambda:100.
                g.process=Process();g.phase='active';g.still_since=99.
                g.ingest('rt/lowstate',{'motor_state':[{'q':0.,'dq':0.,'motorstate':0}]*29})
                g.ingest('rt/secondary_imu',{'quaternion':[1.,0.,0.,0.]})
                g.ingest('g1_debug',dict(encoder_mode=0,motion_name='planner_motion',play=True))
                g.playback.clip.dq.flags.writeable=True;g.playback.clip.dq[0,0]=2.
                result=subprocess.run([sys.executable,str(script),'play','--entry-checks','off',
                                       '--token-file',str(token),
                                       '--url','http://127.0.0.1:%d'%server.server_port],
                                      cwd=folder,capture_output=True,text=True,timeout=5)
                self.assertEqual(result.returncode,0,result.stderr)
                status=json.loads(result.stdout)
                self.assertEqual(status['playback_entry_checks'],'off')
                self.assertEqual(status['playback']['state'],'playing')
                self.assertTrue(status['mode_switch_pending'])
            finally:
                server.shutdown();thread.join(2);server.server_close()

    def test_real_transport_wire_pause_finish_and_feedback(self):
        """Only ephemeral loopback ZMQ; no ROS or real process is created."""
        import msgpack
        import zmq
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);write_clip(root/'short',frames=25)
            g=NativeLifecycle([0.]*29,lambda:Process(),lambda:'Bumpless policy handoff complete',lambda:None)
            g.enable_playback(root)
            g.playback.load(CSVClip(root/'short',fps=50,joint_order='sonic_reference'))
            g.process=Process();g.phase='active';g.still_since=time.monotonic()-1
            # Seed valid feedback before the first active watchdog tick. The
            # periodic loopback source below then owns freshness continuously.
            g.ingest('rt/lowstate',{'motor_state':[{'q':0.,'dq':0.,'motorstate':0}]*29})
            g.ingest('rt/secondary_imu',{'quaternion':[1.,0.,0.,0.]})
            ctx=zmq.Context();state=ctx.socket(zmq.PUB);debug=ctx.socket(zmq.PUB);sink=ctx.socket(zmq.SUB)
            state_port=state.bind_to_random_port('tcp://127.0.0.1')
            # Reserve an adjacent input/feedback pair using real bind checks.
            while True:
                probe=ctx.socket(zmq.PUB);port=probe.bind_to_random_port('tcp://127.0.0.1')
                try:debug.bind('tcp://127.0.0.1:%d'%(port+1))
                except zmq.ZMQError:probe.close(0);continue
                probe.close(0);break
            sink.setsockopt(zmq.SUBSCRIBE,b'');sink.connect('tcp://127.0.0.1:%d'%port)
            transport=Transport(g,state_port,port)
            stop=threading.Event();mode=['planner_motion'];errors=[]
            def feed():
                # This thread owns its sockets; all communication stays on localhost.
                try:
                    while not stop.is_set():
                        state.send_multipart([b'rt/lowstate',msgpack.packb({'motor_state':[{'q':0.,'dq':0.,'motorstate':0}]*29})])
                        state.send_multipart([b'rt/secondary_imu',msgpack.packb({'quaternion':[1.,0.,0.,0.]})])
                        debug.send(b'g1_debug'+msgpack.packb(dict(encoder_mode=0,motion_name=mode[0],play=True)))
                        stop.wait(.01)
                except Exception as exc:errors.append(exc)
            worker=threading.Thread(target=feed)
            transport.thread.start();worker.start()
            try:
                self.assertTrue(transport.ready.wait(2))
                deadline=time.monotonic()+3
                while (not g.robot.safe(g.telemetry()) or not g.policy_feedback) and time.monotonic()<deadline:time.sleep(.01)
                self.assertTrue(g.robot.safe(g.telemetry()))
                def action(action,seq):
                    with g.lock:
                        return g.request('/api/playback',dict(action=action,client='loopback',seq=seq,epoch=g.epoch,ticket=g.ticket()))
                action('play',1)
                mode[0]='streamed'
                deadline=time.monotonic()+2;first=None
                while time.monotonic()<deadline:
                    if sink.poll(50):
                        raw=sink.recv()
                        if raw.startswith(b'pose'):first=unpack(raw)[1]
                    if first is not None and not g.mode_pending:break
                self.assertIsNotNone(first);self.assertFalse(g.mode_pending)
                action('pause',2)
                position=g.playback.position
                time.sleep(.2)
                self.assertEqual(g.playback.position,position)
                action('resume',3)
                deadline=time.monotonic()+3
                while g.playback.state!='finished' and time.monotonic()<deadline:time.sleep(.01)
                self.assertEqual(g.playback.state,'finished')
                self.assertTrue(g.mode_pending)
                mode[0]='planner_motion'
                deadline=time.monotonic()+1
                while g.mode_pending and time.monotonic()<deadline:time.sleep(.01)
                self.assertFalse(g.mode_pending)
                self.assertEqual(g.input_mode,'planner')
                self.assertIsNone(transport.error)
            finally:
                stop.set();worker.join(2);transport.stop.set();transport.thread.join(2)
                sink.close(0);debug.close(0);state.close(0);ctx.term()
            self.assertFalse(errors)


if __name__ == '__main__':unittest.main()
