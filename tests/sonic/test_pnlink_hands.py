"""Hand math and authority tests; synthetic data, no SDK or robot output."""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import numpy as np
from scipy.spatial.transform import Rotation
from onboardmimic.backends.sonic.pnlink.hand_mapping import Mapping
from onboardmimic.backends.sonic.pnlink.hands import PNLinkHands
from onboardmimic.backends.sonic.pico.hands import PicoHandLink, TOPIC


def calibration():
    return dict(version=2,hand='right',open=np.zeros((15,3)).tolist(),closed=np.full((15,3),.3).tolist())


def raw(now=100.,frame=1,pose=None):
    q=Rotation.from_rotvec(np.zeros((15,3)) if pose is None else pose).as_quat()[:,[3,0,1,2]]
    return dict(receiver_id='a',device=dict(serial='device'),avatar=dict(index=0),calibration_revision=0,
                data_source='sdk',receiver_running=True,capture_state='capturing',received_monotonic=now,frame_id=frame,
                right_hand=dict(valid=True,frame_id=frame,received_monotonic=now,rotations_wxyz=q.tolist()))


class MappingTest(unittest.TestCase):
    def test_reference_golden_v1_v2_modes_and_amplitudes(self):
        data=json.loads((Path(__file__).parents[1]/'fixtures/pnlink_hand_parity.json').read_text())
        for c in data['cases']:
            m=Mapping(data['calibrations'][c['version']-1],c['mode'],c['closure'])
            for pose,target in zip(data['poses'],c['targets']):np.testing.assert_allclose(m.target(pose),target,atol=1e-12)
    def test_independent_fingers_thumb_and_amplitude(self):
        m=Mapping(calibration(),'fingers',.25)
        p=np.zeros((15,3));p[12:]=.3
        np.testing.assert_allclose(m.target(p),[.15,.1875,0,0,0,0],atol=1e-12)
        for joint,channel in ((0,2),(3,3),(9,4),(6,5)):
            p=np.zeros((15,3));p[joint:joint+3]=.3
            expected=np.zeros(6);expected[channel]=.25
            np.testing.assert_allclose(m.target(p),expected,atol=1e-12)
    def test_missing_channel_rejects_fingers_but_grasp_needs_two(self):
        c=calibration();c['closed'][6:9]=np.zeros((3,3)).tolist()
        with self.assertRaisesRegex(ValueError,'pinky'):Mapping(c,'fingers')
        m=Mapping(c,'grasp',.25);np.testing.assert_allclose(m.target(c['closed']),[.15,.1875,.25,.25,.25,.25])
        c['closed'][:12]=np.zeros((12,3)).tolist()
        with self.assertRaises(ValueError):Mapping(c,'grasp')


class HandsTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.path=Path(self.tmp.name)/'settings.json'
        self.now=100.;self.data=raw();self.reader=NS(snapshot=lambda:dict(raw=self.data))
        self.h=PNLinkHands(self.reader,self.path,lambda:self.now);self.h.update()
        self.h.calibration=calibration();self.h.origin=dict(device=self.data['device'],avatar=self.data['avatar'])
        self.h.confirm(True);self.h.configure('fingers',.25)
        self.worker=Mock();self.worker.sequence=0;self.worker.clock_ns=lambda:round(self.now*1e9)
        self.status=dict(fresh=True,bridge_id='b'*32,observe_only=False,finish_open_supported=True,intent_protocols=[1,2],sides=dict(right=dict(ready=True)))
        self.worker.status=lambda:self.status
        self.g=NS(phase='active',pnlink_owned=lambda:True,mode_pending=False,task_active=lambda:False,pnlink_session=NS(paused=False))
    def feed(self,pose=None,step=.02):
        self.now+=step;self.data=raw(self.now,self.data['frame_id']+1,pose);self.h.update()
    def packet(self):
        p=self.h.message(self.g,self.worker)
        return json.loads(p[len(TOPIC):]) if p else None
    def test_restart_retains_choice_but_cannot_send_without_entry_and_confirmation(self):
        h=PNLinkHands(self.reader,self.path,lambda:self.now);h.update()
        self.assertEqual((h.mode,h.closure),('fingers',.25));self.assertFalse(h.session);self.assertIsNone(h.confirmed)
        self.assertEqual(self.path.stat().st_mode&0o777,0o600)
        self.assertFalse(self.packet()['active']);self.h.enter();self.assertTrue(self.packet()['active'])
    def test_change_requires_exit_but_off_immediately_revokes(self):
        self.h.enter()
        with self.assertRaises(ValueError):self.h.configure('grasp',.25,owned=True)
        with self.assertRaises(ValueError):self.h.configure('off',.5,owned=True)
        self.h.configure('off',.25,owned=True);self.assertFalse(self.packet()['active'])
        for mode,closure in [('bad',.25),('fingers',True),('fingers',float('nan')),('fingers',.01)]:
            with self.assertRaises(ValueError):self.h.configure(mode,closure)
    def test_pause_freezes_goal_resume_and_exit_never_command_left(self):
        self.h.enter();a=self.packet();self.assertIsNone(a['left'])
        self.g.pnlink_session.paused=True;self.feed(np.full((15,3),.3));b=self.packet()
        self.assertEqual(a['right'],b['right']);self.g.pnlink_session.paused=False
        self.assertNotEqual(a['right'],self.packet()['right']);self.h.revoke();self.assertFalse(self.packet()['active'])
    def test_input_timeout_does_not_revive_same_session(self):
        self.h.enter();self.assertTrue(self.packet()['active']);self.now+=.251
        self.assertFalse(self.packet()['active']);self.feed();self.assertFalse(self.packet()['active'])
        self.h.enter();self.assertTrue(self.packet()['active'])
    def test_invalid_hand_and_feedback_faults_are_latched(self):
        self.h.enter();self.data['right_hand']['valid']=False;self.h.update();self.assertFalse(self.packet()['active'])
        self.feed();self.h.enter();self.status['sides']['right']['ready']=False
        self.assertFalse(self.packet()['active']);self.status['sides']['right']['ready']=True;self.assertFalse(self.packet()['active'])
    def test_worker_restart_and_observe_only_cannot_send(self):
        self.h.enter();self.packet();self.status['bridge_id']='c'*32
        self.assertFalse(self.packet()['active']);self.h.enter();self.status['observe_only']=True
        self.assertFalse(self.packet()['active'])
    def test_duplicate_frame_cannot_refresh_age(self):
        self.h.enter();self.packet();self.now+=.26
        self.data['right_hand']['received_monotonic']=self.now;self.data['received_monotonic']=self.now
        self.h.update();self.assertFalse(self.packet()['active'])
    def test_receiver_change_clears_confirmation_without_deleting_calibration(self):
        self.h.enter();self.packet();self.data['receiver_id']='new';self.h.update()
        self.assertIsNone(self.h.confirmed);self.assertIsNotNone(self.h.calibration);self.assertFalse(self.packet()['active'])
        self.h.confirm(True);self.h.enter();self.assertTrue(self.packet()['active'])
    def test_sdk_calibration_revision_invalidates_persisted_samples(self):
        self.data['calibration_revision']=1;self.h.update()
        self.assertIsNone(self.h.calibration);self.assertIsNone(json.loads(self.path.read_text())['calibration'])
    def capture(self,kind,pose,finger=None):
        self.h.start_capture(kind,finger)
        for _ in range(104):self.feed(pose)
    def test_capture_two_poses_then_single_finger_and_open_clears_closed(self):
        self.capture('open',np.zeros((15,3)));self.assertEqual(self.h.capture_status['stage'],'succeeded')
        self.assertNotIn('closed',self.h.calibration)
        self.capture('closed',np.full((15,3),.3));self.assertEqual(self.h.capture_status['stage'],'succeeded')
        before=copy.deepcopy(self.h.calibration);p=np.zeros((15,3));p[6:9]=.4
        self.capture('finger',p,'pinky');self.assertEqual(self.h.calibration['closed'],before['closed'])
        self.assertIn('pinky',self.h.calibration['finger_closed']);self.assertTrue(self.h.status()['calibration_confirmed'])
        self.capture('open',np.zeros((15,3)));self.assertNotIn('closed',self.h.calibration);self.assertIsNone(self.h.confirmed)
    def test_unstable_capture_fails_without_replacing_saved_calibration(self):
        before=copy.deepcopy(self.h.calibration);self.h.start_capture('open')
        for i in range(105):self.feed(np.full((15,3),.3 if i%2 else 0.))
        self.assertEqual(self.h.capture_status['stage'],'failed');self.assertEqual(self.h.calibration,before)
    def test_capture_disconnect_cancel_and_new_device_requires_open(self):
        self.h.start_capture('open');self.data['right_hand']['valid']=False;self.h.update()
        self.assertIsNone(self.h.capture);self.assertEqual(self.h.capture_status['stage'],'failed')
        self.feed();self.data['device']={'serial':'other'};self.h.update()
        with self.assertRaises(ValueError):self.h.confirm(True)
        with self.assertRaises(ValueError):self.h.start_capture('closed')
    def test_malformed_rotations_do_not_retain_previous_pose(self):
        self.data['right_hand']['rotations_wxyz']=[[float('nan')]*4]*15;self.h.update();self.assertFalse(self.h.input_ready())
    def test_fault_survives_stop_and_is_not_replaced_by_operator_reason(self):
        self.h.enter();self.packet();self.status['sides']['right'].update(ready=False,age_ms=501,samples=123)
        self.assertFalse(self.packet()['active']);reason=self.h.error
        self.h.revoke('控制器已停止')
        self.assertEqual(self.h.error,reason);self.assertEqual(self.h.last_fault['reason'],reason)
        self.assertEqual(self.h.last_fault['worker']['feedback_age_ms'],501)
        self.h.enter();self.assertEqual(self.h.error,'');self.assertEqual(self.h.last_fault['reason'],reason)
    def test_expired_worker_gate_is_reported_instead_of_silent_preview_only(self):
        self.h.enter();self.packet()
        self.status.update(last_session=self.h.session,active=False,error='手部意图超过250ms未更新')
        self.assertFalse(self.packet()['active']);self.assertIn('250ms',self.h.error)
        self.status.update(error='',active=True);self.assertFalse(self.packet()['active'])
    def test_explicit_finish_opens_only_owned_session_and_waits_for_feedback_ack(self):
        self.status['finish_open_supported']=True
        self.h.enter();self.packet();owned=self.h.session
        self.status['last_session']=owned
        self.h.revoke('主动退出',open_hand=True)
        packet=self.packet();self.assertFalse(packet['active']);self.assertTrue(packet['finish_open'])
        self.assertEqual(packet['session'],owned);self.assertIsNone(packet['right'])
        with self.assertRaises(ValueError):self.h.enter()
        with self.assertRaises(ValueError):self.h.configure('grasp',.25)
        self.status['sides']['right'].update(finish_session=owned,finish_stage='opening')
        self.packet();self.assertEqual(self.h.finish_task['stage'],'opening')
        self.status['sides']['right'].update(finish_stage='succeeded',finish_error='')
        self.assertIsNone(self.packet());self.assertFalse(self.h.finishing());self.assertEqual(self.h.finish_task['stage'],'succeeded')
        self.h.enter();self.assertIsNone(self.h.finish_task)
    def test_fault_exit_does_not_open_but_explicit_end_after_fault_can(self):
        self.status['finish_open_supported']=True
        self.h.enter();self.packet();self.status['last_session']=self.h.session
        self.h.fail('输入断流');self.h.revoke('异常退出')
        self.assertIsNone(self.h.finish_task);self.assertNotIn('finish_open',self.packet())
        self.h.revoke('主动急停',open_hand=True);self.assertTrue(self.packet()['finish_open'])
    def test_cleanup_failure_does_not_retry_or_drive_new_worker(self):
        self.status['finish_open_supported']=True
        self.h.enter();self.packet();self.h.revoke(open_hand=True)
        self.status['sides']['right']['ready']=False
        self.assertIsNone(self.packet());self.assertEqual(self.h.finish_task['stage'],'failed')
        self.status['sides']['right']['ready']=True
        self.assertNotIn('finish_open',self.packet())
    def test_idle_exit_never_creates_cleanup_and_fault_cancels_running_cleanup(self):
        self.h.revoke(open_hand=True);self.assertIsNone(self.h.finish_task)
        self.h.enter();self.packet();self.h.revoke(open_hand=True)
        self.assertTrue(self.h.finishing());self.h.revoke('身体故障')
        self.assertFalse(self.h.finishing());self.assertEqual(self.h.finish_task['stage'],'failed')
    def test_hand_mapping_exception_disables_only_intent(self):
        self.h.enter();self.h.mapping=Mock(side_effect=RuntimeError('bad hand math'))
        self.assertFalse(self.packet()['active']);self.assertTrue(self.h.blocked)


class BilateralHandsTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'settings.json';self.now=100.;self.frame=0
        self.data={};self.reader=NS(snapshot=lambda:dict(raw=self.data))
        self.h=PNLinkHands(self.reader,self.path,lambda:self.now)
        self.worker=NS(sequence=0,clock_ns=lambda:round(self.now*1e9),status=lambda:self.status)
        self.status=dict(fresh=True,bridge_id='b'*32,observe_only=False,finish_open_supported=True,
            continuous_sides=['left','right'],intent_protocols=[1,2],sides={s:dict(ready=True) for s in ('left','right')})
        self.g=NS(phase='active',pnlink_owned=lambda:True,mode_pending=False,task_active=lambda:False,pnlink_session=NS(paused=False))
        self.feed()
        for side,h in self.h.sides.items():
            h.calibration=calibration();h.calibration['hand']=side
            # Left measured curl has an opposite rotation: learn it, do not mirror right goals.
            if side=='left':h.calibration['closed']=np.full((15,3),-.4).tolist()
            h.origin=dict(device=self.data['device'],avatar=self.data['avatar']);h.confirm(True);h.save()
        self.h.configure('fingers',.25,hand='both')
    def feed(self,left=None,right=None):
        self.now+=.02;self.frame+=1;self.data=raw(self.now,self.frame,right)
        self.data['left_hand']=raw(self.now,self.frame,left)['right_hand'];self.h.update()
    def packet(self):
        p=self.h.message(self.g,self.worker)
        return json.loads(p[len(TOPIC):]) if p else None
    def test_asymmetric_targets_are_independent_and_pause_freezes_both(self):
        left=np.zeros((15,3));left[0:3]=-.4
        right=np.zeros((15,3));right[6:9]=.3
        self.feed(left,right);self.h.enter();a=self.packet()
        np.testing.assert_allclose(a['left'],[0,0,.25,0,0,0]);np.testing.assert_allclose(a['right'],[0,0,0,0,0,.25])
        self.g.pnlink_session.paused=True;self.feed(np.full((15,3),-.4),np.full((15,3),.3))
        b=self.packet();self.assertEqual((a['left'],a['right']),(b['left'],b['right']))
        self.g.pnlink_session.paused=False;c=self.packet()
        self.assertNotEqual(a['left'],c['left']);self.assertNotEqual(a['right'],c['right'])
    def test_one_bad_side_latches_both_without_emitting_partial_goals(self):
        self.h.enter();self.assertTrue(self.packet()['active'])
        self.status['sides']['left']['ready']=False
        p=self.packet();self.assertFalse(p['active']);self.assertIsNone(p['left']);self.assertIsNone(p['right'])
        self.assertIn('左手',self.h.error)
        self.status['sides']['left']['ready']=True;self.assertFalse(self.packet()['active'])
        self.h.enter();self.assertTrue(self.packet()['active'])
    def test_single_hand_does_not_require_the_unselected_side(self):
        self.h.configure('grasp',.25,hand='left');self.data['right_hand']['valid']=False;self.h.update()
        self.h.enter();p=self.packet();self.assertTrue(p['active']);self.assertIsNone(p['right']);self.assertIsNotNone(p['left'])
        self.h.revoke();self.feed();self.h.configure('fingers',.25,hand='right')
        self.data['left_hand']['valid']=False;self.h.update();self.h.enter();p=self.packet()
        self.assertTrue(p['active']);self.assertIsNone(p['left']);self.assertIsNotNone(p['right'])
    def test_no_right_calibration_fallback_and_old_worker_rejects_left(self):
        self.h.left.calibration=None;self.h.left.confirmed=None;self.h.enter()
        self.assertFalse(self.packet()['active']);self.assertIn('左手',self.h.error)
        self.status.pop('continuous_sides');self.h.enter();self.assertFalse(self.packet()['active'])
        self.assertIn('双手',self.h.error)
    def test_both_capture_and_per_side_resample_never_replace_other_hand(self):
        self.h.capture_hands('both','open')
        for _ in range(104):self.feed()
        self.assertFalse(self.h.sampling())
        for side,h in self.h.sides.items():
            self.assertEqual(h.capture_status['stage'],'succeeded');self.assertEqual(h.calibration['hand'],side)
            self.assertNotIn('closed',h.calibration)
        self.h.capture_hands('both','closed')
        for _ in range(104):self.feed(np.full((15,3),-.4),np.full((15,3),.3))
        before=copy.deepcopy(self.h.calibration)
        self.h.capture_hands('left','finger','pinky')
        for _ in range(104):self.feed(np.full((15,3),-.5))
        self.assertEqual(self.h.calibration,before);self.assertIn('pinky',self.h.left.calibration['finger_closed'])
        self.assertTrue(self.h.left.preview()['fingers']['ready'])
    def test_bilateral_capture_preflight_rolls_back_and_checks_side(self):
        self.data['right_hand']['valid']=False;self.h.update()
        with self.assertRaises(ValueError):self.h.capture_hands('both','open')
        self.assertFalse(self.h.sampling());self.assertIsNone(self.h.left.capture)
        with self.assertRaises(ValueError):self.h.capture_hands('invalid','open')
        with self.assertRaises(ValueError):self.h.configure('fingers',.25,hand='invalid')
    def test_sdk_recalibration_invalidates_both_files_and_confirmation(self):
        self.data['calibration_revision']=1;self.h.update()
        for side,h in self.h.sides.items():
            self.assertIsNone(h.calibration);self.assertIsNone(h.confirmed)
            self.assertIsNone(json.loads(h.path.read_text())['calibration'])
    def test_restart_preserves_selection_and_independent_samples_without_auto_entry(self):
        h=PNLinkHands(self.reader,self.path,lambda:self.now);h.update()
        self.assertEqual(h.selected,'both');self.assertEqual(h.mode,'fingers');self.assertFalse(h.session)
        self.assertEqual(h.left.calibration['hand'],'left');self.assertEqual(h.calibration['hand'],'right')
        self.assertIsNone(h.left.confirmed);self.assertIsNone(h.confirmed)
    def test_finish_waits_for_both_feedback_ack_and_never_replays(self):
        self.h.enter();self.packet();session=self.h.session;self.status['last_session']=session
        self.h.revoke(open_hand=True);p=self.packet();self.assertEqual(p['finish_sides'],['left','right'])
        self.status['sides']['left'].update(finish_session=session,finish_stage='succeeded')
        self.status['sides']['right'].update(finish_session=session,finish_stage='opening')
        self.assertTrue(self.packet()['finish_open']);self.assertEqual(self.h.finish_task['stage'],'opening')
        self.status['sides']['right']['finish_stage']='succeeded'
        self.assertIsNone(self.packet());self.assertEqual(self.h.finish_task['stage'],'succeeded')
        self.h.revoke(open_hand=True);self.assertNotIn('finish_open',self.packet())
    def test_left_only_finish_and_lost_feedback_cancel(self):
        self.h.configure('fingers',.25,hand='left');self.h.enter();self.packet();self.h.revoke(open_hand=True)
        self.assertEqual(self.packet()['finish_sides'],['left'])
        self.status['sides']['left']['ready']=False
        self.assertIsNone(self.packet());self.assertEqual(self.h.finish_task['stage'],'failed')
        self.status['sides']['left']['ready']=True;self.assertNotIn('finish_open',self.packet())
    def test_mirrored_calibration_cannot_be_loaded_for_other_side(self):
        self.h.left.path.write_text(self.path.read_text())
        h=PNLinkHands(self.reader,self.path,lambda:self.now)
        self.assertIsNone(h.left.calibration);self.assertTrue(h.left.load_error)
        with self.assertRaises(ValueError):Mapping(self.h.calibration,'fingers',hand='left')
    def test_unselected_right_time_regression_cannot_disable_left_only_session(self):
        self.h.configure('fingers',.25,hand='left');self.h.enter();self.assertTrue(self.packet()['active'])
        self.frame-=2;self.feed();self.assertTrue(self.h.input_fault)
        # Both streams regressed here; restore only the selected left stream.
        self.h.left.input_fault='';self.frame+=3;self.feed()
        self.assertTrue(self.packet()['active']);self.assertIsNone(self.packet()['right'])
        self.h.revoke();self.h.configure('fingers',.25,hand='both');self.h.enter();self.packet()
        self.frame-=2;self.feed();self.frame+=3;self.feed()
        self.assertFalse(self.packet()['active']);self.assertIn('时间回退',self.h.error)
