import tempfile
import time
from pathlib import Path
from unittest.mock import patch
import unittest
from tests.runtime import test_native_lifecycle as fixtures
from tests.sonic.test_pnlink import snapshot
from tests.sonic.test_pico_session import PoseReader as PicoReader, PicoSession, feed, controls
from onboardmimic.backends.sonic.pnlink.reader import PoseReader
from onboardmimic.backends.sonic.pnlink.session import PNLinkSession
from onboardmimic.backends.sonic.transport import Transport


class PNControlTest(unittest.TestCase):
    def setUp(self):
        self.h=fixtures.NativeTest();self.h.setUp();self.h.activate();self.g=self.h.g
        self.reader=PoseReader('/unused',lambda:self.h.now)
        self.g.pnlink_session=PNLinkSession(self.reader);self.g.pnlink_socket=Path('/unused.sock')
        self.transport=Transport(self.g);self.live=True;self.generation='a'
        self.advance(.7);self.g.pnlink_session.confirm()
    def advance(self,seconds,ack=True):
        for _ in range(round(seconds/.02)):
            self.h.now+=.02;self.h.refresh()
            if self.live:self.reader.feed(snapshot(self.h.now,receiver=self.generation))
            if self.g.pico_session:feed(self.g.pose_reader,self.h.now,controls(),controls())
            if ack:
                encoder=2 if self.g.input_mode in ('pose','smpl_playback') else 0
                self.g.ingest('g1_debug',dict(encoder_mode=encoder,motion_name='streamed' if self.g.input_mode!='planner' else 'planner_motion',play=True))
            self.g.tick();self.transport.messages()
    def cmd(self,action,**kw):return self.g.request('/api/controller',self.h.packet(action=action,**kw))
    def enter(self):
        self.cmd('select_pnlink',alignment_confirmed=True);self.advance(1.)
        self.assertTrue(self.g.pnlink_owned());self.assertFalse(self.g.mode_pending)
        self.assertEqual(self.g.motion_task['stage'],'finished')
    def test_pn_only_enter_pause_resume_exit_same_pid(self):
        self.assertIsNone(self.g.pose_reader);self.enter()
        self.assertEqual(self.g.status()['input_source'],'pnlink')
        self.assertFalse(self.g.status()['planner_control_allowed'])
        self.cmd('pause_pnlink');self.advance(.3);self.assertTrue(self.g.pnlink_session.paused)
        self.cmd('resume_pnlink');self.cmd('select_planner');self.advance(.7)
        self.assertEqual(self.g.status()['input_source'],'planner');self.assertEqual(self.h.launches,1)
    def test_invalid_target_does_not_revoke_current_source(self):
        self.enter();self.g.pnlink_session.clear_calibration()
        with self.assertRaises(ValueError):self.cmd('select_pnlink',alignment_confirmed=True)
    def test_no_alignment_never_enters(self):
        with self.assertRaises(ValueError):self.cmd('select_pnlink')
        self.assertFalse(self.g.pnlink_owned())
    def test_loss_during_pause_and_recovery_do_not_resume(self):
        self.enter();self.cmd('pause_pnlink');self.live=False;self.advance(.4)
        self.assertFalse(self.g.pnlink_owned());self.assertEqual(self.g.input_mode,'planner')
        self.live=True;self.advance(.8);self.assertFalse(self.g.pnlink_owned())
    def test_receiver_restart_requires_new_confirmation(self):
        self.enter();self.generation='b';self.advance(.5)
        self.assertFalse(self.g.pnlink_owned());self.assertFalse(self.g.pnlink_session.calibrated())
    def test_wrong_mode_ack_times_out_without_restarting_process(self):
        self.cmd('select_pnlink',alignment_confirmed=True);self.advance(.8)
        self.assertTrue(self.g.pnlink_owned());self.advance(2.2,ack=False)
        self.assertFalse(self.g.pnlink_owned());self.assertEqual(self.h.launches,1)
    def test_stop_and_estop_revoke_even_if_reference_recovers(self):
        self.enter();self.g.request('/api/control',self.h.packet(stop=True));self.advance(.7)
        self.assertFalse(self.g.pnlink_owned());self.enter();self.cmd('estop')
        self.assertFalse(self.g.pnlink_owned());self.assertEqual(self.g.phase,'stopped')
    def test_browser_expiry_keeps_active_but_takeover_exits(self):
        self.g.request('/api/operator',self.h.packet(action='acquire',actor='a'*20));self.g.operator_deadline=self.h.now+.1
        # Enter directly through the already-authorized transaction helper.
        self.g.start_pnlink_task(True);self.advance(1.)
        # Unstarted task is cancelled on expiry, so enter afresh without a browser owner.
        self.assertFalse(self.g.pnlink_owned());self.advance(.7);self.enter()
        self.g.expire_operator('browser closed');self.advance(.3);self.assertTrue(self.g.pnlink_owned())
        self.g.request('/api/operator',self.h.packet(action='takeover',actor='b'*20))
        self.assertFalse(self.g.pnlink_owned())
    def test_pico_and_pn_roundtrip_keep_readers_separate(self):
        pico=PicoReader('/unused',lambda:self.h.now);self.g.pose_reader=pico;self.g.pico_session=PicoSession(pico)
        self.advance(.3);self.enter()
        self.cmd('select_pico',pico_mode='pose',alignment_confirmed=True);self.advance(1.)
        self.assertTrue(self.g.pico_owned());self.assertFalse(self.g.pnlink_owned());self.assertIs(self.g.pose_reader,pico)
        self.enter();self.assertFalse(self.g.pico_owned());self.assertEqual(self.h.launches,1)
    def test_calibration_waits_for_zero_planner_and_never_auto_reenters(self):
        self.enter();sent=[]
        with patch.object(self.g.backend,'pnlink_command',side_effect=lambda *args:sent.append(args)):
            self.g.request('/api/inputs/pnlink',self.h.packet(action='calibrate'))
            self.assertFalse(self.g.pnlink_owned());self.assertFalse(sent)
            self.advance(.3);self.assertFalse(sent)
            self.advance(.5)
            for _ in range(100):
                if sent:break
                time.sleep(.001)
            self.assertEqual(sent[0][1],'calibrate');self.assertEqual(self.g.input_mode,'planner')
            self.assertFalse(self.g.pnlink_session.calibrated());self.assertEqual(self.h.launches,1)
    def test_device_commands_and_confirmation_require_operator_and_revision(self):
        self.g.request('/api/operator',self.h.packet(action='acquire',actor='a'*20))
        with self.assertRaisesRegex(ValueError,'操作权'):
            self.g.request('/api/inputs/pnlink',self.h.packet(action='confirm_calibration',calibration_confirmed=True))
    def test_cancel_pending_takeover_and_late_ack_never_enter(self):
        self.g.request('/api/operator',self.h.packet(action='acquire',actor='a'*20))
        self.g.start_pnlink_task(True)
        self.g.request('/api/operator',self.h.packet(action='takeover',actor='a'*20))
        self.advance(1.5);self.assertFalse(self.g.pnlink_owned())
        self.assertEqual(self.g.motion_task['stage'],'cancelled')
    def test_file_roundtrip_and_failed_preflight_keep_live_source(self):
        from tests.runtime.test_pico_control import PicoControlTest
        PicoControlTest.setup_file(self);self.enter()
        with patch.object(self.g.backend,'prepare_clip',side_effect=ValueError('bad target')):
            self.g.request('/api/playback',self.h.packet(action='switch',motion_id='cache/next'))
            PicoControlTest.wait_preflight(self)
        self.assertTrue(self.g.pnlink_owned());self.advance(.1)
        self.g.request('/api/playback',self.h.packet(action='switch',motion_id='cache/next'))
        PicoControlTest.wait_preflight(self);self.advance(1.)
        self.assertEqual(self.g.input_mode,'joint_playback');self.assertFalse(self.g.pnlink_owned())
        self.enter();self.assertEqual(self.h.launches,1)

    def test_smpl_file_roundtrip_has_distinct_source_and_same_pid(self):
        from tests.runtime.test_pico_control import PicoControlTest
        from tests.sonic.test_motion_import import write_smpl
        PicoControlTest.setup_file(self)
        write_smpl(self.g.motion_root/'human.npz',frames=300)
        self.g.library.entries.append(dict(id='cache/human',name='human',motion='human.npz',kind='cache',status='ready'))
        self.enter();self.g.request('/api/playback',self.h.packet(action='switch',motion_id='cache/human'))
        PicoControlTest.wait_preflight(self);self.advance(1.)
        self.assertEqual(self.g.status()['input_source'],'file_smpl');self.assertFalse(self.g.pnlink_owned())
        self.enter();self.assertEqual(self.g.status()['input_source'],'pnlink');self.assertEqual(self.h.launches,1)


if __name__=='__main__':unittest.main()
