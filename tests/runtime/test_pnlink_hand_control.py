"""Gateway integration: optional hand failures cannot stop body transport."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock
from tests.runtime import test_pnlink_control as body_fixture
from tests.sonic.test_pnlink_hands import raw, calibration
from onboardmimic.backends.sonic.pnlink.hands import PNLinkHands
from onboardmimic.backends.sonic.pico.hands import PicoHandLink


class PNHandControlTest(unittest.TestCase):
    def setUp(self):
        self.body=body_fixture.PNControlTest();self.body.setUp();self.g=self.body.g
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.h=PNLinkHands(self.body.reader,Path(self.tmp.name)/'settings.json',lambda:self.body.h.now)
        self.g.pnlink_hands=self.h
    def request(self,action,**kw):return self.g.request('/api/inputs/pnlink',self.body.h.packet(action=action,**kw))
    def test_config_persists_and_cannot_change_while_pn_owned(self):
        self.request('configure_hands',hand_mode='fingers',hand_closure=.25)
        self.body.enter()
        with self.assertRaises(ValueError):self.request('configure_hands',hand_mode='grasp',hand_closure=.25)
        self.request('configure_hands',hand_mode='off',hand_closure=.25)
        self.assertTrue(self.g.pnlink_owned());self.assertEqual(self.body.h.launches,1)
    def test_missing_fingers_never_prevents_body_entry_and_only_hand_faults(self):
        self.request('configure_hands',hand_mode='fingers',hand_closure=.25)
        self.g.pico_hand_link=PicoHandLink(Path(self.tmp.name)/'missing')
        self.body.enter();self.body.advance(.4)
        self.assertTrue(self.h.blocked);self.assertTrue(self.g.pnlink_owned());self.assertEqual(self.g.phase,'active')
        self.assertEqual(self.body.h.proc.kills,0)
    def test_status_and_processing_exceptions_do_not_stop_body(self):
        self.body.enter();self.h.update=Mock(side_effect=RuntimeError('hand-only'))
        self.g.pico_hand_link=Mock();self.g.pico_hand_link.status.side_effect=RuntimeError('no worker')
        self.body.advance(.3);s=self.g.status()
        self.assertEqual(s['state'],'active');self.assertTrue(s['pnlink_hands']['error']);self.assertEqual(self.body.h.proc.kills,0)
    def test_device_calibration_discards_hand_samples_before_dispatch(self):
        self.h.calibration=calibration();self.h.save()
        self.g.backend.pnlink_command=Mock()
        self.request('calibrate');self.assertIsNone(self.h.calibration)
    def test_operator_and_revision_are_required(self):
        self.g.request('/api/operator',self.body.h.packet(action='acquire',actor='a'*20))
        with self.assertRaisesRegex(ValueError,'操作权'):
            self.request('configure_hands',hand_mode='fingers',hand_closure=.25)
        packet=self.body.h.packet(action='configure_hands',hand_mode='fingers',hand_closure=.25,actor='a'*20,operator_generation=self.g.operator_generation,expected_revision=-1)
        with self.assertRaises(ValueError):self.g.request('/api/inputs/pnlink',packet)
    def test_positive_pn_transport_pause_resume_and_optional_fault(self):
        import numpy as np
        from types import SimpleNamespace as NS
        feed=self.body.reader.feed
        self.pose=np.zeros((15,3))
        def with_hands(data):
            data['right_hand']=raw(self.body.h.now,data['frame_id'],self.pose)['right_hand']
            feed(data)
        self.body.reader.feed=with_hands
        self.body.advance(.2);self.h.calibration=calibration()
        self.h.origin=dict(device=self.h.key[1],avatar=self.h.key[2]);self.h.confirm(True)
        self.request('configure_hands',hand_mode='fingers',hand_closure=.25)
        status=dict(fresh=True,bridge_id='b'*32,observe_only=False,finish_open_supported=True,intent_protocols=[1,2],sides=dict(right=dict(ready=True)))
        self.g.pico_hand_link=NS(sequence=0,clock_ns=lambda:round(self.body.h.now*1e9),status=lambda:status)
        self.body.enter()
        def hand():
            messages=self.body.transport.messages();self.assertTrue(messages[0].startswith(b'pose'))
            return json.loads(messages[-1][len(b'pico_hands'):])
        self.assertTrue(hand()['active']);self.assertEqual(hand()['right'],[0]*6)
        self.body.cmd('pause_pnlink');self.pose[:]=.3;self.body.advance(.2)
        self.assertEqual(hand()['right'],[0]*6)
        self.body.cmd('resume_pnlink');self.body.advance(.1)
        np.testing.assert_allclose(hand()['right'],[.15,.1875,.25,.25,.25,.25])
        status['sides']['right']['ready']=False
        self.assertFalse(hand()['active']);self.assertTrue(self.g.pnlink_owned())
        status['sides']['right']['ready']=True;self.assertFalse(hand()['active'])
        self.assertEqual(self.body.h.launches,1);self.assertEqual(self.body.h.proc.kills,0)
    def test_hand_off_is_available_during_body_handoff(self):
        self.request('configure_hands',hand_mode='fingers',hand_closure=.25)
        self.body.cmd('select_pnlink',alignment_confirmed=True)
        self.assertTrue(self.g.task_active())
        self.request('configure_hands',hand_mode='off',hand_closure=.25)
        self.assertEqual(self.h.mode,'off');self.assertTrue(self.g.task_active())
    def test_real_capture_actions_open_closed_and_finger_through_gateway(self):
        import numpy as np
        feed=self.body.reader.feed
        pose=np.zeros((15,3))
        def with_hands(data):
            data['right_hand']=raw(self.body.h.now,data['frame_id'],pose)['right_hand']
            feed(data)
        self.body.reader.feed=with_hands;self.body.advance(.2)
        result=self.request('capture_hand_open')
        self.assertEqual(result['pnlink_hands']['capture']['stage'],'running')
        self.body.advance(2.2)
        self.assertEqual(self.h.capture_status['stage'],'succeeded');self.assertNotIn('closed',self.h.calibration)
        pose[:]=.3
        self.request('capture_hand_closed');self.body.advance(2.2)
        self.assertEqual(self.h.capture_status['stage'],'succeeded');self.assertTrue(self.h.preview()['fingers']['ready'])
        self.request('capture_hand_finger',finger='pinky');self.body.advance(2.2)
        self.assertEqual(self.h.capture_status['stage'],'succeeded');self.assertIn('pinky',self.h.calibration['finger_closed'])
        self.assertEqual(self.body.h.launches,1);self.assertEqual(self.body.h.proc.kills,0)
    def setup_live_hands(self):
        import numpy as np
        from types import SimpleNamespace as NS
        feed=self.body.reader.feed
        def with_hands(data):
            data['right_hand']=raw(self.body.h.now,data['frame_id'],np.full((15,3),.2))['right_hand'];feed(data)
        self.body.reader.feed=with_hands;self.body.advance(.2)
        self.h.calibration=calibration();self.h.origin=dict(device=self.h.key[1],avatar=self.h.key[2]);self.h.confirm(True)
        self.request('configure_hands',hand_mode='fingers',hand_closure=.25)
        self.ws=dict(fresh=True,bridge_id='b'*32,observe_only=False,finish_open_supported=True,intent_protocols=[1,2],sides=dict(right=dict(ready=True)))
        self.g.pico_hand_link=NS(sequence=0,clock_ns=lambda:round(self.body.h.now*1e9),status=lambda:self.ws,message=lambda g:None)
        self.body.enter();self.ws['last_session']=self.h.session
    def test_exit_and_estop_send_cleanup_even_when_body_is_stopped(self):
        self.setup_live_hands();owned=self.h.session
        self.body.cmd('estop');self.assertEqual(self.g.phase,'stopped');self.assertTrue(self.h.finishing())
        packets=self.body.transport.messages();packet=json.loads(packets[-1][len(b'pico_hands'):])
        self.assertTrue(packet['finish_open']);self.assertEqual(packet['session'],owned)
        self.assertEqual(self.body.h.launches,1)
    def test_pause_is_freeze_user_exit_is_open_and_fault_exit_is_hold(self):
        self.setup_live_hands();self.body.cmd('pause_pnlink');self.assertFalse(self.h.finishing())
        self.body.cmd('select_planner');self.assertTrue(self.h.finishing())
        self.ws['sides']['right'].update(finish_session=self.h.finish_task['session'],finish_stage='succeeded')
        self.body.transport.messages();self.body.advance(.7);self.body.enter()
        self.body.live=False;self.body.advance(.4)
        self.assertFalse(self.g.pnlink_owned());self.assertFalse(self.h.finishing())
    def test_cleanup_defers_pico_hand_packets_without_stopping_body(self):
        self.setup_live_hands();self.body.cmd('select_planner')
        called=[];self.g.pico_hand_link.message=lambda g:called.append(True)
        self.body.transport.messages();self.assertFalse(called)
        self.ws['sides']['right'].update(finish_session=self.h.finish_task['session'],finish_stage='succeeded')
        self.body.transport.messages();self.body.transport.messages();self.assertTrue(called)
        self.assertEqual(self.g.phase,'active');self.assertEqual(self.body.h.proc.kills,0)
    def test_sampling_blocked_while_active_and_never_creates_hand_output(self):
        self.body.enter()
        with self.assertRaises(ValueError):self.request('capture_hand_open')
        self.assertIsNone(self.h.capture);self.assertEqual(self.body.h.launches,1)

    def test_bilateral_capture_through_authenticated_gateway_and_busy_interlocks(self):
        import numpy as np
        feed=self.body.reader.feed;pose=np.zeros((15,3))
        def with_hands(data):
            sample=raw(self.body.h.now,data['frame_id'],pose)['right_hand']
            data.update(right_hand=sample,left_hand=dict(sample));feed(data)
        self.body.reader.feed=with_hands;self.body.advance(.2)
        self.request('capture_hand_open',hand='both');self.assertTrue(self.h.sampling())
        with self.assertRaises(ValueError):self.request('calibrate')
        with self.assertRaises(ValueError):self.request('configure_hands',hand_mode='fingers',hand_closure=.25,hand='both')
        self.body.advance(2.2)
        for h in self.h.sides.values():self.assertEqual(h.capture_status['stage'],'succeeded')
        pose[:]=.3;self.request('capture_hand_closed',hand='both');self.body.advance(2.2)
        result=self.request('confirm_hand_calibration',hand='both',calibration_confirmed=True)
        self.assertTrue(all(h['calibration_confirmed'] for h in result['pnlink_hands']['sides'].values()))
        self.assertEqual(self.body.h.proc.kills,0)

    def test_left_capture_and_selection_require_valid_hand_and_never_start_body(self):
        for action in ('capture_hand_open','confirm_hand_calibration'):
            with self.assertRaises(ValueError):self.request(action,hand='bad',calibration_confirmed=True)
        self.request('configure_hands',hand='both',hand_mode='fingers',hand_closure=.25)
        self.assertEqual(self.h.selected,'both');self.body.enter()
        with self.assertRaises(ValueError):self.request('configure_hands',hand='left',hand_mode='fingers',hand_closure=.25)
        self.assertEqual(self.body.h.launches,1)
