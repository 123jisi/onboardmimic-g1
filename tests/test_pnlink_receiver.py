"""Device-free tests of handle ownership, calibration results and SDK isolation."""
import ctypes
import tempfile
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from onboardmimic.inputs.pnlink.receiver import Receiver
from onboardmimic.inputs.pnlink.state import atomic_write, read_state
from onboardmimic.inputs.pnlink.install_service import render


class FakeSDK:
    MCPBvhData=NS(Binary=1)
    MCPBvhDisplacement=NS(Enable=1)
    MCPBvhRotation=NS(YXZ=2)
    MCPError=NS(NoError=0)
    EMCPCommand=NS(CommandStartCapture=0,CommandStopCapture=1,CommandCalibrateMotion=3)
    MCPEventType=NS(AvatarUpdated=256,Notify=1792,CommandReply=1536,Error=768)
    MCPEventNotify=NS(Notify_SystemUpdated=3)
    MCPReplay=NS(MCPReplay_Running=1,MCPReplay_Result=2)
    MCPCalibrateMotionProgressStep=NS(CalibrateMotionProgressStep_Countdown=1,CalibrateMotionProgressStep_Progress=2)
    result=0
    destroyed=[]
    class MCPSettings:
        def __getattr__(self,_):return lambda *args:None
    class MCPApplication:
        def __init__(self):self._handle=1;self.opened=False;self.events=[];self.queued=[];self.api=NS(contents=NS(QueuedServerCommand=self.queue))
        def queue(self,handle,app):self.queued.append(handle.value);return 0
        def set_settings(self,s):pass
        def open(self):self.opened=True;return True,'ok'
        def is_opened(self):return self.opened
        def close(self):self.opened=False;return True,'ok'
        def poll_next_event(self):events,self.events=self.events,[];return events
    class MCPCommand:
        def __init__(self):self.handle=ctypes.c_uint64(42)
        def create_command(self,t):pass
        def get_result_code(self,h):return FakeSDK.result
        def get_result_message(self,h):return 'device failed'
        def get_progress(self,h):return 7
        def destroy_command(self,h):FakeSDK.destroyed.append(h)
    class MCPCalibrateMotionProgress:
        def __init__(self,h):pass
        def get_step_current_pose(self):return 2,'T-pose'
        def get_progress_current_pose(self):return 100,'T-pose'
    class MCPAvatar:
        def __init__(self,h):pass
        def get_index(self):return 0
        def get_name(self):return 'human'
        def get_joints(self):
            return [NS(get_name=lambda:'Hips',get_local_rotation=lambda:(1.,0.,0.,0.),get_local_position=lambda:(0.,0.,0.))]


class ReceiverTest(unittest.TestCase):
    def setUp(self):
        self.now=100.;FakeSDK.result=0;FakeSDK.destroyed=[]
        self.r=Receiver(FakeSDK,'192.0.2.11','192.0.2.20',clock=lambda:self.now);self.r.open()
    def submit(self,action='start_capture',key='a'):
        return self.r.submit(dict(action=action,receiver_id=self.r.id,request_id=key))
    def reply(self,phase=2,handle=42):self.r.reply(NS(_commandHandle=handle,_replay=phase))
    def avatar(self,timestamp=100.):self.r.avatar(NS(timestamp=timestamp,event_data=NS(avatar_handle=1)))
    def test_open_does_not_start_capture_and_closes_cleanly(self):
        self.assertEqual(self.r.app.queued,[]);self.assertEqual(self.r.data['frame_id'],0)
        self.r.close();self.assertTrue(self.r.data['sdk_open_succeeded']);self.assertTrue(self.r.data['sdk_closed_cleanly'])
    def test_repeated_request_is_idempotent_and_new_commands_serialized(self):
        self.submit();self.submit();self.assertEqual(len(self.r.app.queued),1)
        with self.assertRaises(ValueError):self.submit('stop_capture','b')
        with self.assertRaises(ValueError):self.submit('stop_capture','a')
        self.reply(handle=99);self.assertEqual(self.r.pending,'start_capture')
        self.reply();self.assertIsNone(self.r.pending);self.assertEqual(FakeSDK.destroyed,[42])
    def test_calibration_progress_100_is_not_success_and_timeout_is_not_cancel(self):
        self.r.data.update(connected=True,capture_state='capturing');self.submit('calibrate')
        self.reply(phase=1);self.assertEqual(self.r.data['command']['progress'],100)
        self.assertEqual(self.r.data['calibration']['evidence'],'unknown')
        self.now+=91;self.r.poll();self.assertEqual(self.r.data['command']['stage'],'unknown')
        self.assertIsNotNone(self.r.pending)
        self.reply();self.assertEqual(self.r.data['calibration']['evidence'],'sdk_command_succeeded')
    def test_calibration_failure_never_proves_validity(self):
        self.r.data.update(connected=True,capture_state='capturing');self.submit('calibrate')
        FakeSDK.result=3;self.reply();self.assertEqual(self.r.data['command']['stage'],'failed')
        self.assertEqual(self.r.data['calibration']['evidence'],'unknown')
    def test_duplicate_and_regressed_source_time(self):
        self.avatar();received=self.r.data['received_monotonic'];self.now+=1;self.avatar()
        self.assertEqual(self.r.data['received_monotonic'],received)
        with self.assertRaises(RuntimeError):self.avatar(1.)
    def test_missing_or_broken_optional_fingers_never_gate_body(self):
        from unittest.mock import patch
        body=FakeSDK.MCPAvatar(1).get_joints()
        broken=NS(get_name=lambda:'LeftHandIndex1',get_local_rotation=lambda:(_ for _ in ()).throw(ValueError('finger unavailable')))
        with patch.object(FakeSDK.MCPAvatar,'get_joints',return_value=body+[broken]):self.avatar()
        self.assertEqual(len(self.r.data['joints']),1)
    def test_right_fingers_are_optional_ordered_and_local_rotation_only(self):
        from unittest.mock import patch
        from onboardmimic.inputs.pnlink.model import RIGHT_HAND_NAMES
        body=FakeSDK.MCPAvatar(1).get_joints()
        fingers=[NS(get_name=lambda n=n:n,get_local_rotation=lambda:(1.,0.,0.,0.)) for n in reversed(RIGHT_HAND_NAMES)]
        with patch.object(FakeSDK.MCPAvatar,'get_joints',return_value=body+fingers):self.avatar()
        h=self.r.data['right_hand'];self.assertTrue(h['valid']);self.assertEqual(len(h['rotations_wxyz']),15)
        self.assertEqual(len(self.r.data['joints']),1)
        with patch.object(FakeSDK.MCPAvatar,'get_joints',return_value=body+fingers[:-1]):self.avatar(101.)
        self.assertFalse(self.r.data['right_hand']['valid']);self.assertEqual(self.r.data['right_hand']['rotations_wxyz'],[])
    def test_broken_right_finger_does_not_break_body_frame(self):
        from unittest.mock import patch
        body=FakeSDK.MCPAvatar(1).get_joints()
        broken=NS(get_name=lambda:'RightHandIndex1',get_local_rotation=lambda:None)
        with patch.object(FakeSDK.MCPAvatar,'get_joints',return_value=body+[broken]):self.avatar()
        self.assertEqual(len(self.r.data['joints']),1);self.assertFalse(self.r.data['right_hand']['valid'])
    def test_late_avatar_after_stop_does_not_restart_capture(self):
        self.submit('stop_capture');self.reply();self.avatar()
        self.assertEqual(self.r.data['capture_state'],'stopped')
    def test_shutdown_with_pending_command_marks_unknown_not_cancelled(self):
        self.submit();self.r.close();self.assertEqual(self.r.data['command']['stage'],'unknown')
    def test_wrong_receiver_and_unsupported_cancel_rejected(self):
        with self.assertRaises(ValueError):self.r.submit(dict(action='start_capture',receiver_id='old',request_id='a'))
        with self.assertRaises(ValueError):self.submit('cancel')
    def test_sdk_error_exits_instead_of_claiming_connected(self):
        self.r.app.events=[NS(event_type=FakeSDK.MCPEventType.Error,event_data=NS(error=2))]
        with self.assertRaises(RuntimeError):self.r.poll()
    def test_private_atomic_snapshot_and_receive_only_unit(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'status.json';atomic_write(path,self.r.data)
            self.assertEqual(path.stat().st_mode&0o777,0o600);self.assertEqual(read_state(path)['receiver_id'],self.r.id)
        unit=render(Path('/opt/onboard'),dict(local_ip='192.0.2.11',server_ip='192.0.2.20'))
        self.assertIn('NoNewPrivileges=true',unit);self.assertIn('inputs.pnlink.receiver',unit)
        self.assertNotIn('--physical',unit);self.assertNotIn('core.runtime',unit)


if __name__=='__main__':unittest.main()


class BilateralReceiverTest(unittest.TestCase):
    setUp = ReceiverTest.setUp
    avatar = ReceiverTest.avatar
    def test_left_and_right_order_and_validity_are_independent(self):
        from unittest.mock import patch
        from onboardmimic.inputs.pnlink.model import HAND_NAMES
        body=FakeSDK.MCPAvatar(1).get_joints()
        joints=body[:]
        for side,names in HAND_NAMES.items():
            for i,n in enumerate(names):
                q=(1.,(i+1)*.001 if side=='left' else 0.,0.,0.)
                joints.append(NS(get_name=lambda n=n:n,get_local_rotation=lambda q=q:q))
        with patch.object(FakeSDK.MCPAvatar,'get_joints',return_value=list(reversed(joints))):self.avatar()
        self.assertTrue(self.r.data['left_hand']['valid']);self.assertTrue(self.r.data['right_hand']['valid'])
        self.assertEqual(self.r.data['left_hand']['rotations_wxyz'][0],[1.,.001,0.,0.])
        self.assertEqual(self.r.data['left_hand']['rotations_wxyz'][-1],[1.,.015,0.,0.])
        missing=[j for j in joints if j.get_name()!='LeftHandThumb3']
        with patch.object(FakeSDK.MCPAvatar,'get_joints',return_value=missing):self.avatar(101.)
        self.assertFalse(self.r.data['left_hand']['valid']);self.assertTrue(self.r.data['right_hand']['valid'])
        self.assertEqual(len(self.r.data['joints']),1)
    def test_duplicate_left_joint_is_invalid_without_hiding_right_or_body(self):
        from unittest.mock import patch
        from onboardmimic.inputs.pnlink.model import HAND_NAMES
        joints=FakeSDK.MCPAvatar(1).get_joints()+[NS(get_name=lambda n=n:n,get_local_rotation=lambda:(1.,0.,0.,0.)) for names in HAND_NAMES.values() for n in names]
        joints.append(NS(get_name=lambda:'LeftHandIndex1',get_local_rotation=lambda:(1.,0.,0.,0.)))
        with patch.object(FakeSDK.MCPAvatar,'get_joints',return_value=joints):self.avatar()
        self.assertFalse(self.r.data['left_hand']['valid']);self.assertTrue(self.r.data['right_hand']['valid'])
