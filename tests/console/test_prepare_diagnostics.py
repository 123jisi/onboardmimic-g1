"""Admission explanations must describe live flags without admitting motion."""
import unittest
from onboardmimic.core.connection import prepare_blockers
from onboardmimic.robots.unitree_g1.telemetry import feedback, safe


class PrepareDiagnosticsTest(unittest.TestCase):
    def state(self, **changes):
        state=dict(state='stopped',pids=[],setup_ready=True,prepare_available=False,
            telemetry=dict(fresh=True,valid=True,faults=[],max_speed_rad_s=0.),
            feedback_continuity=dict(ready=True))
        state.update(changes)
        return state

    def test_raw_motor_fault_is_explained_without_reinterpreting_firmware_code(self):
        motors=[dict(q=0.,dq=0.,motorstate=0,temperature=[40,41]) for _ in range(29)]
        motors[5].update(motorstate=512,temperature=[42,42])
        latest={'rt/lowstate':(10.,dict(motor_state=motors)),
                'rt/secondary_imu':(10.,dict(quaternion=[1.,0.,0.,0.]))}
        telemetry=feedback(latest,[0.]*29,10.01)
        self.assertEqual(telemetry['faults'],[5]);self.assertFalse(safe(telemetry))
        issues=prepare_blockers(self.state(telemetry=telemetry))
        self.assertEqual(len(issues),1);issue=issues[0]
        self.assertEqual(issue['code'],'motor_fault');self.assertEqual(issue['index'],5)
        self.assertEqual(issue['motorstate'],512)
        self.assertIn('左踝横滚',issue['message']);self.assertIn('0x200',issue['message'])
        self.assertIn('[42, 42]',issue['message'])
        self.assertNotIn('过热',issue['message'])

    def test_all_current_failures_are_reported_and_stale_faults_are_not_current(self):
        state=self.state(setup_ready=False,setup_error='连接服务失败：桥接错误',pids=[123],
            telemetry=dict(fresh=False,valid=True,faults=[5]),feedback_continuity=dict(ready=False))
        issues=prepare_blockers(state)
        self.assertEqual({x['code'] for x in issues},{'services','controller_running','feedback_stale','feedback_continuity'})
        self.assertIn('桥接错误',issues[0]['message'])

    def test_invalid_feedback_and_high_speed_have_specific_reasons(self):
        self.assertEqual(prepare_blockers(self.state(telemetry=dict(fresh=True,valid=False)))[0]['code'],'feedback_invalid')
        state=self.state(telemetry=dict(fresh=True,valid=True,faults=[],max_speed_rad_s=13.))
        self.assertEqual(prepare_blockers(state)[0]['code'],'joint_speed')

    def test_unexplained_backend_block_is_visible_and_healthy_state_is_empty(self):
        self.assertEqual(prepare_blockers(self.state())[0]['code'],'backend')
        self.assertEqual(prepare_blockers(self.state(prepare_available=True)),[])

    def test_existing_feedback_without_fault_details_still_shows_index(self):
        state=self.state(telemetry=dict(fresh=True,valid=True,faults=[5]))
        issues=prepare_blockers(state)
        self.assertIn('电机 5',issues[0]['message'])
        self.assertIsNone(issues[0]['motorstate'])
