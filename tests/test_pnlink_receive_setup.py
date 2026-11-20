"""Manual start, late device power-up, and no robot/calibration automation."""
import tempfile
from pathlib import Path
from unittest.mock import Mock, patch
import unittest
from tests.test_pnlink_receiver import FakeSDK
from tests.runtime import test_pnlink_control as fixtures
from onboardmimic.inputs.pnlink.receiver import Receiver
from onboardmimic.inputs.pnlink.autocapture import CaptureReconnect
from onboardmimic.inputs.pnlink.service import change
from onboardmimic.inputs.pnlink.install_service import render


class CaptureStartTest(unittest.TestCase):
    def setUp(self):
        FakeSDK.result=0;FakeSDK.destroyed=[]
        self.now=100.
        self.receiver=Receiver(FakeSDK,'192.0.2.10','192.0.2.20',clock=lambda:self.now,auto_capture=True)
        self.receiver.open()

    def test_explicit_service_start_requests_only_capture_after_delay(self):
        self.receiver.poll();self.assertEqual(self.receiver.app.queued,[])
        self.now+=2.1;self.receiver.poll()
        self.assertEqual(self.receiver.pending,'start_capture')
        self.assertTrue(self.receiver.data['command']['automatic'])
        self.assertNotEqual(self.receiver.pending,'calibrate')

    def test_default_receiver_still_sends_no_commands(self):
        receiver=Receiver(FakeSDK,'a','b',clock=lambda:self.now);receiver.open()
        self.now+=100;receiver.poll();self.assertEqual(receiver.app.queued,[])

    def test_late_device_reconnects_only_own_start_command(self):
        self.now+=3;self.receiver.poll();self.now+=21
        with self.assertRaises(CaptureReconnect):self.receiver.poll()
        receiver=Receiver(FakeSDK,'a','b',clock=lambda:self.now,auto_capture=True);receiver.open()
        self.now+=3;receiver.poll();self.assertEqual(receiver.pending,'start_capture')

    def test_manual_stop_is_not_undone(self):
        self.receiver.submit(dict(action='stop_capture',receiver_id=self.receiver.id,request_id='manual'))
        self.receiver.reply(type('Reply',(),{'_commandHandle':42,'_replay':2})())
        self.now+=100;self.receiver.poll()
        self.assertTrue(self.receiver.auto_capture.suspended)
        self.assertEqual(self.receiver.app.queued,[42])

    def test_manual_calibration_is_never_restarted_on_timeout(self):
        self.receiver.data.update(connected=True,capture_state='capturing')
        self.receiver.submit(dict(action='calibrate',receiver_id=self.receiver.id,request_id='manual'))
        self.now+=120;self.receiver.poll()
        self.assertEqual(self.receiver.pending,'calibrate')
        self.assertEqual(self.receiver.data['command']['stage'],'unknown')

    def test_existing_fresh_frames_do_not_need_start_command(self):
        self.receiver.data.update(capture_state='capturing',received_monotonic=self.now)
        self.receiver.poll();self.assertEqual(self.receiver.app.queued,[])

    def test_unit_opt_in_and_service_control_have_no_robot_entrypoint(self):
        root=Path('/opt/onboard')
        base=dict(local_ip='192.0.2.10',server_ip='192.0.2.20')
        self.assertNotIn('--auto-capture',render(root,base))
        unit=render(root,dict(base,auto_capture=True));self.assertIn('--auto-capture',unit)
        self.assertIn('StartLimitIntervalSec=0',unit);self.assertNotIn('core.runtime',unit)
        runner=Mock(return_value=Mock(returncode=0,stderr=''))
        change('start_receiver',runner)
        self.assertEqual(runner.call_args.args[0],['systemctl','--user','start','onboardmimic-pnlink-receiver.service'])
        with self.assertRaises(ValueError):change('restart_robot',runner)


class ReceiverGatewayTest(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.PNControlTest();self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.g=self.fixture.g;self.h=self.fixture.h

    def test_start_receiver_does_not_need_live_sdk_and_never_starts_robot(self):
        import time
        self.g.pnlink_session.reader.receiver_id=None
        with patch.object(self.g.backend,'pnlink_service') as service:
            self.g.request('/api/inputs/pnlink',self.h.packet(action='start_receiver'))
            for _ in range(100):
                if self.g.pnlink_command['stage']!='sending':break
                time.sleep(.001)
            service.assert_called_once_with('start_receiver')
            self.assertFalse(self.g.pnlink_busy());self.assertFalse(self.g.pnlink_owned())
            self.assertEqual(self.h.launches,1)

    def test_stop_active_receiver_waits_for_planner_barrier(self):
        self.fixture.enter()
        with patch.object(self.g.backend,'pnlink_service') as service:
            self.g.request('/api/inputs/pnlink',self.h.packet(action='stop_receiver'))
            self.assertFalse(self.g.pnlink_owned());service.assert_not_called()
            self.fixture.advance(.3);service.assert_not_called()
            self.fixture.advance(.5)
            import time
            for _ in range(100):
                if service.called:break
                time.sleep(.001)
            service.assert_called_once_with('stop_receiver')
            self.assertEqual(self.g.input_mode,'planner');self.assertEqual(self.h.launches,1)

    def test_service_operations_require_same_operator(self):
        self.g.request('/api/operator',self.h.packet(action='acquire',actor='a'*20))
        with self.assertRaisesRegex(ValueError,'操作权'):
            self.g.request('/api/inputs/pnlink',self.h.packet(action='start_receiver'))


if __name__=='__main__':unittest.main()
