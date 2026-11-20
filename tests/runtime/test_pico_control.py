"""Fake-process, measured-feedback integration; never talks to a robot."""
import unittest
from pathlib import Path
import tempfile
from tests.runtime import test_native_lifecycle as fixtures
from tests.sonic.test_pico_session import PoseReader,PicoSession,feed,controls
from onboardmimic.backends.sonic.transport import Transport

class PicoControlTest(unittest.TestCase):
    def setUp(self):
        self.h=fixtures.NativeTest();self.h.setUp();self.h.activate();self.g=self.h.g
        self.left=controls();self.right=controls();self.reader=PoseReader('/unused',lambda:self.h.now)
        self.g.pose_reader=self.reader;self.g.pico_session=PicoSession(self.reader)
        self.transport=Transport(self.g);self.live_left=True
        self.advance(.7)

    def advance(self,duration,ack=True):
        for _ in range(round(duration/.02)):
            self.h.now+=.02;self.h.refresh()
            feed(self.reader,self.h.now,self.left,self.right,left_stamp=None if self.live_left else self.reader.controls['left']['stamp'])
            if ack:
                starting=self.g.task_active() and self.g.motion_task['stage']=='starting'
                p=self.g.pico_session
                active=p.active and (not self.g.task_active() or self.g.motion_task.get('kind')!='pico' or starting)
                encoder=2 if active and p.mode=='pose' else 1 if active and p.mode=='vr3pt' else 0
                if self.g.input_mode=='smpl_playback':encoder=2
                streamed=encoder==2 or self.g.input_mode=='joint_playback'
                self.g.ingest('g1_debug',dict(encoder_mode=encoder,motion_name='streamed' if streamed else 'planner_motion',play=True))
            self.g.tick()
            self.transport.messages()

    def request(self,mode='pose',**kw):
        return self.g.request('/api/controller',self.h.packet(action='select_pico',pico_mode=mode,alignment_confirmed=True,**kw))

    def enter(self,mode='pose'):
        self.request(mode);self.advance(1.)
        self.assertEqual(self.g.motion_task['stage'],'finished')
        self.assertTrue(self.g.pico_owned());self.assertFalse(self.g.mode_pending)
        self.advance(.02)  # Neutral/release baseline is consumed by the session.

    def test_every_mode_has_real_ack_and_same_process(self):
        for mode in ('pose','planner','frozen','vr3pt'):
            self.request(mode);self.assertEqual(self.g.motion_task['stage'],'waiting_planner')
            self.assertTrue(self.g.mode_pending);self.advance(1.)
            self.assertEqual(self.g.motion_task['stage'],'finished')
            self.assertEqual(self.g.status()['input_source'],'pico')
            self.assertEqual(self.g.status()['pico_mode'],mode)
            self.assertFalse(self.g.status()['planner_control_allowed'])
            self.assertEqual(self.h.launches,1)
            self.assertTrue(self.transport.messages())

    def test_hand_intent_uses_same_gateway_authority_and_is_revoked_on_exit(self):
        import json
        from onboardmimic.backends.sonic.pico.hands import PicoHandLink,TOPIC
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        path=Path(temp.name)/'hands.json'
        def hand_packet():
            path.write_text(json.dumps(dict(protocol=1,running=True,bridge_id='a'*32,
                updated_monotonic_ns=round(self.h.now*1e9))))
            return json.loads(self.transport.messages()[-1][len(TOPIC):])
        self.g.pico_hand_link=PicoHandLink(path,lambda:round(self.h.now*1e9))
        self.assertFalse(hand_packet()['active'])
        self.enter('planner');self.assertTrue(hand_packet()['active'])
        self.left['trigger']=1.;self.advance(.02);self.assertTrue(hand_packet()['left'])
        old=self.g.pico_session.hand_session
        self.g.request('/api/control',self.h.packet(stop=True));self.advance(.7)
        self.assertFalse(hand_packet()['active']);self.assertNotEqual(self.g.pico_session.hand_session,old)

    def test_missing_stale_or_unavailable_hands_never_gate_body_modes(self):
        import json
        from onboardmimic.backends.sonic.pico.hands import PicoHandLink
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        path=Path(temp.name)/'hands.json'
        self.g.pico_hand_link=PicoHandLink(path,lambda:round(self.h.now*1e9))
        for mode in ('pose','planner','frozen','vr3pt'):
            self.enter(mode)  # Worker status is absent; body entry must succeed.
            path.write_text(json.dumps(dict(protocol=1,running=True,bridge_id='a'*32,
                updated_monotonic_ns=round(self.h.now*1e9),
                sides=dict(left=dict(ready=False),right=dict(ready=False)))))
            messages=self.transport.messages()
            self.assertTrue(messages[0].startswith(b'pose' if mode=='pose' else b'planner'))
            self.assertTrue(self.g.pico_owned());self.assertEqual(self.g.phase,'active')
            self.advance(.7)  # Status expires while body feedback and PICO continue.
            path.unlink();self.advance(.3)
            self.assertTrue(self.g.pico_owned());self.assertEqual(self.g.phase,'active')
        self.assertEqual(self.h.launches,1);self.assertEqual(self.h.proc.kills,0)

    def test_hand_message_exception_disables_only_hands_until_explicit_reentry(self):
        from unittest.mock import Mock
        self.enter('planner');link=Mock();self.g.pico_hand_link=link
        link.message.side_effect=RuntimeError('hand-only failure')
        self.assertTrue(self.transport.messages()[0].startswith(b'planner'))
        self.advance(.3)
        self.assertTrue(self.g.pico_owned());self.assertEqual(self.g.phase,'active')
        self.assertEqual(link.message.call_count,1)
        self.assertTrue(self.g.pico_hand_error)
        link.message.side_effect=None;link.message.return_value=b'pico_hands-test'
        self.enter('planner')  # Explicit web re-entry resets the hand session ID.
        self.assertGreater(link.message.call_count,1)
        self.assertEqual(self.g.pico_hand_error,'');self.assertEqual(self.h.proc.kills,0)

    def test_hand_status_exception_is_reported_without_stopping_body(self):
        from unittest.mock import Mock
        self.enter('planner');self.g.pico_hand_link=Mock()
        self.g.pico_hand_link.status.side_effect=RuntimeError('unavailable hand status')
        status=self.g.status()
        self.assertEqual(status['state'],'active');self.assertEqual(status['input_source'],'pico')
        self.assertFalse(status['pico_hands']['fresh']);self.assertTrue(status['pico_hands']['error'])

    def test_invalid_optional_hand_packet_cannot_break_body_publisher(self):
        from unittest.mock import Mock
        self.enter('planner');self.g.pico_hand_link=Mock()
        for invalid in (True,42,b'planner-invalid-hand-packet'):
            self.g.pico_session.reset_hands()
            self.g.pico_hand_link.message.return_value=invalid
            messages=self.transport.messages()
            self.assertEqual(len(messages),1);self.assertTrue(messages[0].startswith(b'planner'))
            self.assertEqual(self.g.phase,'active');self.assertTrue(self.g.pico_hand_error)
    def test_body_cannot_keep_disconnected_stick_alive(self):
        self.enter('planner');self.live_left=False;self.advance(.3)
        self.assertTrue(self.g.pico_owned());self.assertTrue(self.g.pico_session.recovering)
        self.advance(3.1)
        self.assertFalse(self.g.pico_owned());self.assertEqual(self.g.input_mode,'planner')
        self.live_left=True;self.advance(1.)
        self.assertFalse(self.g.pico_owned());self.assertEqual(self.g.last_packet['speed'],0)
    def test_both_index_triggers_preserve_body_authority_in_every_mode(self):
        for mode in ('pose','planner','frozen','vr3pt'):
            self.left['trigger']=self.right['trigger']=0.;self.enter(mode)
            self.g.pico_session.hand_intent()  # Establish the per-side release baseline.
            for side in (self.left,self.right):
                side['trigger']=1.;self.advance(.4)
                self.assertTrue(self.g.pico_owned());self.assertEqual(self.g.pico_session.mode,mode)
                self.assertTrue(self.g.pico_session.hand_intent()['left' if side is self.left else 'right'])
                side['trigger']=0.;self.advance(.1)
            self.assertEqual(self.h.proc.kills,0)

    def test_controller_loss_logs_specific_stream_without_pose_payload(self):
        self.enter('planner');self.live_left=False
        with self.assertLogs('runtime_audit',level='WARNING') as logs:self.advance(.3)
        import json
        event=json.loads(logs.records[0].message)
        self.assertEqual(event['event'],'pico_tracking_hold')
        self.assertTrue(event['tracking']['body_ready'])
        self.assertIn('left',event['controller_errors'])
        self.assertNotIn('right',event['controller_errors'])
        self.assertNotIn('payload',json.dumps(event))
        self.assertTrue(self.g.pico_owned());self.assertTrue(self.g.pico_session.recovering)

    def tracking_gap(self,duration,ack=True):
        """Robot feedback stays live, but no tracking source produces samples."""
        for _ in range(round(duration/.02)):
            self.h.now+=.02;self.h.refresh()
            if ack:
                p=self.g.pico_session
                encoder=2 if p.active and p.mode=='pose' else 1 if p.active and p.mode=='vr3pt' else 0
                self.g.ingest('g1_debug',dict(encoder_mode=encoder,
                    motion_name='streamed' if encoder==2 else 'planner_motion',play=True))
            self.g.tick();self.transport.messages()

    def test_short_gap_holds_same_mode_and_resumes_without_handoff(self):
        for mode in ('pose','planner','frozen','vr3pt'):
            with self.subTest(mode=mode):
                self.enter(mode);p=self.g.pico_session
                epoch=self.g.epoch;hand_session=p.hand_session
                self.tracking_gap(.6)
                self.assertTrue(self.g.pico_owned());self.assertTrue(p.recovering)
                self.assertEqual(self.g.input_mode,'pose' if mode=='pose' else 'planner')
                self.assertFalse(self.g.mode_pending);self.assertEqual(self.g.epoch,epoch)
                messages=self.transport.messages()
                self.assertEqual(len(messages),1)
                self.assertTrue(messages[0].startswith(b'pose' if mode=='pose' else b'planner'))
                self.advance(.6)
                self.assertTrue(self.g.pico_owned());self.assertFalse(p.recovering)
                self.assertEqual(p.mode,mode);self.assertEqual(self.g.epoch,epoch)
                self.assertNotEqual(p.hand_session,hand_session)
                self.assertEqual(self.h.launches,1);self.assertEqual(self.h.proc.kills,0)

    def test_pose_hold_flattens_last_frame_and_advances_only_output_clock(self):
        import numpy as np
        from tests.sonic.test_pico_session import unpack
        self.enter();p=self.g.pico_session
        p.cached_reference['smpl_pose'][:,0,0]=np.arange(5)*.01
        p.cached_reference['smpl_joints'][:,0,0]=np.arange(5)*.01
        p.cached_reference['joint_vel'][:]=1.
        source_stamp=self.reader.prev[0]
        self.tracking_gap(.6)
        first=unpack(self.transport.messages()[0]);self.tracking_gap(.04)
        second=unpack(self.transport.messages()[0])
        for name in ('smpl_pose','smpl_joints','body_quat_w','joint_pos'):
            np.testing.assert_allclose(first[name],np.repeat(first[name][-1:],5,axis=0))
            np.testing.assert_allclose(first[name],second[name])
        np.testing.assert_array_equal(first['joint_vel'],0)
        np.testing.assert_array_equal(first['heading_increment'],0)
        self.assertGreater(second['frame_index'][-1],first['frame_index'][-1])
        self.assertGreater(second['timestamp_monotonic'][0],first['timestamp_monotonic'][0])
        self.assertEqual(self.reader.prev[0],source_stamp)
        self.assertFalse(self.reader.snapshot()['ready'])

    def test_planner_hold_stops_walk_and_turn_without_dropping_upper_targets(self):
        import numpy as np
        from tests.runtime.test_local_playback import unpack
        for mode in ('planner','frozen','vr3pt'):
            self.left=controls();self.right=controls();self.enter(mode)
            p=self.g.pico_session;p.gait=1
            self.left['axes']=[0,1];self.right['axes']=[.4,0];self.advance(.1)
            previous=dict(p.cached_reference)
            self.assertGreater(previous['speed'],0)
            self.tracking_gap(.6)
            _,fields=unpack(self.transport.messages()[0],topic=b'planner')
            self.assertEqual(fields['mode'][0],0);self.assertEqual(fields['speed'][0],0)
            np.testing.assert_array_equal(fields['movement'],0)
            heading=p.heading;self.tracking_gap(.1)
            self.assertEqual(p.heading,heading)
            for wire,key in (('upper_body_position','upper_body_position'),('vr_position','vr_3pt_position'),('vr_orientation','vr_3pt_orientation')):
                if key in previous:np.testing.assert_allclose(fields[wire],previous[key])
            self.advance(.5);self.assertTrue(p.recovering)  # A held stick cannot reissue motion.
            self.left['axes']=self.right['axes']=[0,0];self.advance(.5)
            self.assertTrue(p.active);self.assertFalse(p.recovering)

    def test_timeout_revokes_once_and_later_data_cannot_restart(self):
        self.enter();self.tracking_gap(3.5)
        self.assertFalse(self.g.pico_owned());self.assertEqual(self.g.input_mode,'planner')
        self.assertIn('超时',self.g.pico_session.error)
        self.advance(1.)
        self.assertFalse(self.g.pico_owned());self.assertEqual(self.h.launches,1)
        self.enter('planner')  # Explicit re-entry is still available.

    def test_loss_between_tick_and_publication_or_pose_watchdog_still_holds(self):
        self.enter();epoch=self.g.epoch
        self.h.now+=.3;self.h.refresh()
        messages=self.transport.messages()
        self.assertTrue(self.g.pico_session.recovering)
        self.assertEqual(len(messages),1);self.assertTrue(messages[0].startswith(b'pose'))
        self.g.tick_mode(self.h.now)
        self.assertEqual(self.g.epoch,epoch);self.assertEqual(self.g.input_mode,'pose')
        self.assertTrue(self.g.pico_owned())

    def test_new_receiver_or_clock_fault_never_automatically_recovers(self):
        self.enter('planner')
        feed(self.reader,self.h.now+.01,receiver_id='restarted')
        self.g.tick();self.assertFalse(self.g.pico_owned())
        self.advance(.6);self.enter('planner')
        feed(self.reader,self.h.now+.01,left_stamp=1)
        self.g.tick();self.assertFalse(self.g.pico_owned())

    def test_recovery_requires_continuous_freshness_and_keeps_original_deadline(self):
        self.enter('planner');self.live_left=False;self.advance(.4)
        p=self.g.pico_session;deadline=p.recovery_since+p.recovery_seconds
        for _ in range(5):
            self.live_left=True;self.advance(.1)
            self.live_left=False;self.advance(.3)
            if not p.active:break
        self.assertTrue(p.recovering);self.assertEqual(p.recovery_since+p.recovery_seconds,deadline)
        self.advance(1.2)
        self.assertFalse(p.active)

    def test_pose_jump_waits_for_alignment_then_expires(self):
        from tests.sonic.test_pico_session import body
        from scipy.spatial.transform import Rotation as R
        self.enter();self.tracking_gap(.6)
        changed=body();changed[0,3:]=(R.from_euler('y',60,degrees=True)*R.from_quat(changed[0,3:])).as_quat()
        for _ in range(35):
            self.h.now+=.02;self.h.refresh();feed(self.reader,self.h.now,poses=changed)
            self.g.ingest('g1_debug',dict(encoder_mode=2,motion_name='streamed',play=True))
            self.g.tick();self.transport.messages()
        self.assertTrue(self.g.pico_session.recovering)
        self.assertIn('姿态',self.g.pico_session.status()['recovery_message'])
        self.tracking_gap(3.)
        self.assertFalse(self.g.pico_owned())

    def test_stop_takeover_and_feedback_loss_override_hold(self):
        self.enter();self.tracking_gap(.6)
        self.g.request('/api/control',self.h.packet(stop=True))
        self.advance(.5);self.assertFalse(self.g.pico_owned())
        self.enter();self.tracking_gap(.6)
        self.g.operator_request(dict(action='takeover',actor='b'*20))
        self.advance(.5);self.assertFalse(self.g.pico_owned())
        self.g.operator_request(dict(action='release',actor='b'*20,operator_generation=self.g.operator_generation))
        self.enter();self.tracking_gap(.6);self.tracking_gap(.6,ack=False)
        self.assertFalse(self.g.pico_owned());self.assertEqual(self.h.proc.kills,0)

    def test_hand_intent_stays_inactive_until_body_recovery_finishes(self):
        import json
        from onboardmimic.backends.sonic.pico.hands import PicoHandLink,TOPIC
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        path=Path(temp.name)/'hands.json';p=self.g.pico_session
        self.g.pico_hand_link=PicoHandLink(path,lambda:round(self.h.now*1e9))
        def packet():
            path.write_text(json.dumps(dict(protocol=1,running=True,bridge_id='a'*32,updated_monotonic_ns=round(self.h.now*1e9))))
            return json.loads(self.transport.messages()[-1][len(TOPIC):])
        self.enter('planner');self.tracking_gap(.6)
        self.assertFalse(packet()['active'])
        self.advance(.1);self.assertFalse(packet()['active'])
        self.advance(.5);self.assertTrue(packet()['active'])
        self.assertTrue(p.active)

    def test_loss_diagnostic_uses_same_snapshot_as_safety_decision(self):
        from unittest.mock import patch
        import json
        self.enter('planner');snapshot=self.reader.snapshot()
        snapshot.update(controllers_ready=False,controller_errors={'left':'stale'})
        with patch.object(self.reader,'snapshot',return_value=snapshot) as read:
            with self.assertLogs('runtime_audit',level='WARNING') as logs:
                self.g.tick_pico(self.h.now)
        self.assertEqual(read.call_count,1)
        self.assertFalse(json.loads(logs.records[0].message)['controllers_ready'])
    def test_browser_expiry_continues_pico_but_takeover_revokes(self):
        self.g.operator_request(dict(action='acquire',actor='a'*20))
        self.request(actor='a'*20,operator_generation=self.g.operator_generation,expected_revision=self.g.revision)
        self.advance(6.)
        self.assertTrue(self.g.pico_owned());self.assertIsNone(self.g.operator)
        self.g.operator_request(dict(action='takeover',actor='b'*20))
        self.assertFalse(self.g.pico_owned());self.advance(.5)
        self.left['primary']=self.right['primary']=True;self.advance(.1)
        self.assertFalse(self.g.pico_owned())
    def test_planner_request_rejected_during_pico_planner(self):
        self.enter('planner')
        with self.assertRaises(ValueError):self.g.request('/api/control',self.h.packet(move='forward'))
        self.g.request('/api/control',self.h.packet(stop=True));self.assertFalse(self.g.pico_owned())
    def test_alignment_and_controller_preflight_preserve_old_source(self):
        with self.assertRaises(ValueError):
            self.g.request('/api/controller',self.h.packet(action='select_pico'))
        self.assertFalse(self.g.mode_pending)
        self.h.now+=.3;self.h.refresh()
        with self.assertRaises(ValueError):self.request()
        self.assertFalse(self.g.pico_owned())
    def test_wrong_encoder_does_not_ack_vr_mode(self):
        self.request('vr3pt')
        while self.g.motion_task['stage']!='starting':self.advance(.02)
        self.assertEqual(self.g.motion_task['stage'],'starting')
        # Suppress correct feedback, repeatedly send Planner encoder 0.
        for _ in range(110):
            self.advance(.02,ack=False)
            self.g.ingest('g1_debug',dict(encoder_mode=0,motion_name='planner_motion',play=True))
        self.assertFalse(self.g.pico_owned());self.assertEqual(self.g.input_mode,'planner')
        self.advance(.5);self.assertEqual(self.h.proc.kills,0)
    def test_abxy_can_stop_during_mode_barrier_never_restart(self):
        self.enter();self.left['primary']=self.right['primary']=True;self.advance(.02)
        self.assertTrue(self.g.task_active())
        self.left['secondary']=self.right['secondary']=True;self.advance(.02)
        self.assertEqual(self.g.phase,'stopped');self.assertEqual(self.h.proc.kills,1)
        self.advance(.3);self.assertEqual(self.h.launches,1)
    def test_cancel_pending_entry_never_arms_later(self):
        self.request();self.g.cancel_motion_task();self.advance(1.)
        self.assertFalse(self.g.pico_owned());self.assertEqual(self.g.motion_task['stage'],'cancelled')
    def test_no_pico_reference_during_planner_barrier(self):
        self.enter();self.request('vr3pt')
        messages=self.transport.messages()
        self.assertTrue(messages[0].startswith(b'planner'))
        self.assertFalse(any(x.startswith(b'pose') for x in messages))
    def test_active_mode_feedback_loss_reverts(self):
        self.enter('planner');self.advance(.6,ack=False)
        self.assertFalse(self.g.pico_owned());self.assertTrue(self.g.mode_pending)
    def test_pause_resume_failure_reverts_without_restart(self):
        from tests.sonic.test_pico_session import body
        from scipy.spatial.transform import Rotation as R
        self.enter();self.transport.messages();self.left['menu']=True;self.advance(.02)
        self.assertTrue(self.g.pico_session.paused)
        self.left['menu']=False;b=body();b[0,3:]=(R.from_euler('y',60,degrees=True)*R.from_quat(b[0,3:])).as_quat()
        for _ in range(8):
            self.h.now+=.02;self.h.refresh();feed(self.reader,self.h.now,self.left,self.right,poses=b)
        self.g.tick();self.assertFalse(self.g.pico_owned());self.assertEqual(self.h.launches,1)

    def setup_file(self):
        from tests.runtime.test_local_playback import write_clip
        from onboardmimic.backends.sonic.motion_worker import execute
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);root=Path(temp.name)
        write_clip(root/'next',frames=300)
        execute(dict(action='register',root=str(root),motion='next',fps=50,joint_order='sonic_reference',root_body_index=0),root)
        self.g.enable_playback(root)
        self.g.library.entries=[dict(id='cache/next',name='next',motion='next',kind='cache',status='ready')]

    def wait_preflight(self):
        import time
        for _ in range(100):
            if self.g.motion_task['stage']!='checking':return
            time.sleep(.01)
        self.fail('preflight worker stuck')

    def test_pico_file_roundtrip_same_pid_and_manual_reentry(self):
        self.setup_file();self.enter()
        self.g.request('/api/playback',self.h.packet(action='switch',motion_id='cache/next'))
        self.wait_preflight();self.advance(1.)
        self.assertFalse(self.g.pico_owned());self.assertEqual(self.g.input_mode,'joint_playback')
        self.assertEqual(self.g.motion_task['stage'],'finished')
        self.request('vr3pt');self.advance(1.)
        self.assertTrue(self.g.pico_owned());self.assertEqual(self.g.pico_session.mode,'vr3pt')
        self.assertEqual(self.h.launches,1)

    def test_failed_file_preflight_preserves_live_pico_stream(self):
        import threading
        from unittest.mock import patch
        self.setup_file();self.enter();entered=threading.Event();release=threading.Event()
        def bad(*args):
            entered.set();release.wait(2);raise ValueError('broken target')
        with patch.object(self.g.backend,'prepare_clip',side_effect=bad):
            self.g.request('/api/playback',self.h.packet(action='switch',motion_id='cache/next'))
            self.assertTrue(entered.wait(1));self.advance(.1)
            self.assertTrue(self.g.pico_owned());self.assertTrue(self.transport.messages()[0].startswith(b'pose'))
            release.set();self.wait_preflight()
        self.advance(.1);self.assertTrue(self.g.pico_owned());self.assertEqual(self.g.motion_task['stage'],'error')
