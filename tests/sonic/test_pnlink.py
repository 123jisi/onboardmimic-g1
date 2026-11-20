import copy
import json
from pathlib import Path
import unittest
import threading
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation as R
from onboardmimic.backends.sonic.pnlink.mapping import BODY_NAMES, map_joints
from onboardmimic.backends.sonic.pnlink.reader import PoseReader
from onboardmimic.backends.sonic.pnlink.session import PNLinkSession
from tests.runtime.test_local_playback import unpack


def snapshot(now, frame=None, receiver='receiver-a', angle=0):
    joints = [dict(name=n, rotation_wxyz=[1.,0.,0.,0.], position_xyz=[0.,0.,0.]) for n in BODY_NAMES]
    q=R.from_rotvec([angle,0,0]).as_quat()
    joints[16]['rotation_wxyz']=q[[3,0,1,2]].tolist()
    return dict(protocol=1,data_source='sdk',receiver_running=True,receiver_id=receiver,
                updated_monotonic=now,received_monotonic=now,source_timestamp=now,
                frame_id=frame if frame is not None else round(now*10000),joints=joints,
                capture_state='capturing',command={},device=dict(serial='suit-a'),avatar=dict(index=0,name='human'))


class PNMappingTest(unittest.TestCase):
    def setUp(self):
        self.now=100.
        self.reader=PoseReader('/unused',lambda:self.now)
    def feed(self,angle=0,receiver='receiver-a'):
        for _ in range(8):
            self.now+=.02;self.reader.feed(snapshot(self.now,angle=angle,receiver=receiver))

    def test_host_golden_fixture_on_cpu(self):
        fixture=json.loads((Path(__file__).parents[1]/'fixtures/pnlink_host_parity.json').read_text())
        for sample in fixture['samples']:
            actual=map_joints(sample['joints'])
            for key,value in sample['expected'].items():
                value=np.asarray(value)
                if key=='body_quat_w' and np.dot(actual[key],value)<0:value=-value
                np.testing.assert_allclose(actual[key],value,atol=1e-5,rtol=0)

    def test_only_terminal_toes_may_be_derived_as_reference_identity(self):
        rows=snapshot(self.now)['joints']
        without=[r for r in rows if r['name'] not in ('LeftTiptoe','RightTiptoe')]
        for key,value in map_joints(without).items():np.testing.assert_array_equal(value,map_joints(rows)[key])
        with self.assertRaises(ValueError):map_joints([r for r in without if r['name']!='Head'])

    def test_root_wrap_keeps_quaternion_hemisphere_continuous(self):
        previous=None
        for angle in np.linspace(0,2*np.pi,100):
            self.now+=.02;data=snapshot(self.now)
            data['joints'][0]['rotation_wxyz']=R.from_rotvec([0,angle,0]).as_quat()[[3,0,1,2]].tolist()
            self.reader.feed(data);fields=self.reader.snapshot().get('fields')
            if fields is None:continue
            current=fields['body_quat_w'][-1]
            if previous is not None:self.assertGreaterEqual(np.dot(previous,current),0)
            previous=current

    def test_missing_duplicate_invalid_and_extra_joints(self):
        data=snapshot(self.now)
        expected=map_joints(data['joints'])
        data['joints'].append(dict(name='Unknown',rotation_wxyz=[0,0,0,0]))
        for key,value in map_joints(data['joints']).items():np.testing.assert_array_equal(value,expected[key])
        for joints in (data['joints'][1:],data['joints']+[data['joints'][0]],copy.deepcopy(data['joints'])):
            if len(joints)==len(data['joints']):joints[0]['rotation_wxyz']=[float('nan')]*4
            with self.assertRaises(ValueError):map_joints(joints)

    def test_duplicates_cannot_refresh_and_stale_frame_cannot_reanimate(self):
        self.feed();self.assertTrue(self.reader.snapshot()['ready'])
        data=copy.deepcopy(self.reader.raw);self.now+=.3
        data['received_monotonic']=self.now;self.reader.feed(data)
        self.assertFalse(self.reader.snapshot()['ready'])

    def test_regression_locks_generation_and_restart_clears_window(self):
        self.feed();bad=snapshot(self.now+.01);bad['source_timestamp']=1
        self.now+=.01;self.reader.feed(bad)
        self.feed();self.assertFalse(self.reader.snapshot()['ready'])
        self.feed(receiver='receiver-b');self.assertTrue(self.reader.snapshot()['ready'])

    def test_calibration_and_capture_command_invalidate(self):
        self.feed();data=snapshot(self.now+.02);data['command']={'stage':'running'}
        self.now+=.02;self.reader.feed(data);self.assertFalse(self.reader.snapshot()['ready'])
        self.feed();data=snapshot(self.now+.02);data['capture_state']='stopped'
        self.now+=.02;self.reader.feed(data);self.assertFalse(self.reader.snapshot()['ready'])

    def test_pause_keeps_pose_but_advances_protocol_and_resume_requires_alignment(self):
        self.feed();session=PNLinkSession(self.reader)
        with self.assertRaises(ValueError):session.enter()
        session.confirm();session.enter();session.reference(self.now);session.pause()
        old=unpack(session.reference(self.now))[1];self.feed(angle=.6)
        held=unpack(session.reference(self.now))[1]
        np.testing.assert_array_equal(old['smpl_pose'],held['smpl_pose'])
        self.assertGreater(held['frame_index'][0],old['frame_index'][0])
        self.assertGreater(held['timestamp_monotonic'][0],old['timestamp_monotonic'][0])
        with self.assertRaisesRegex(ValueError,'偏差'):session.resume()
        self.feed();session.resume();self.assertFalse(session.paused)

    def test_pause_freezes_final_pose_across_entire_encoder_window(self):
        for angle in np.linspace(0,.3,12):
            self.now+=.02;self.reader.feed(snapshot(self.now,angle=angle))
        session=PNLinkSession(self.reader);session.confirm();session.enter()
        before=unpack(session.reference(self.now))[1]
        self.assertGreater(np.ptp(before['smpl_pose'][:,15,0]),0)
        session.pause();held=unpack(session.reference(self.now+.02))[1]
        for key in ('smpl_pose','smpl_joints','body_quat_w','joint_pos'):
            np.testing.assert_array_equal(held[key],np.repeat(before[key][-1:],5,axis=0))

    def test_restart_and_changed_device_require_manual_confirmation(self):
        self.feed();session=PNLinkSession(self.reader);session.confirm();self.assertTrue(session.calibrated())
        self.feed(receiver='receiver-b');self.assertFalse(session.calibrated())
        session.confirm();data=snapshot(self.now+.02,receiver='receiver-b');data['device']['serial']='another'
        self.now+=.02;self.reader.feed(data);self.assertFalse(session.calibrated())

    def test_poll_loop_survives_realtime_wait_failure_and_observes_new_generation(self):
        class RealtimeWaitEvent(threading.Event):
            def wait(self, timeout=None):
                raise AssertionError('Event.wait may stall after JP5 wall-clock rollback')

        self.reader.stop = RealtimeWaitEvent()
        reads = []

        def receive(_path):
            self.now += .02
            reads.append(self.now)
            if len(reads) == 1:
                # Status file left by the previous boot: future monotonic age.
                return snapshot(self.now + 10000, receiver='previous-boot')
            if len(reads) == 10:
                self.reader.stop.set()
            return snapshot(self.now, receiver='current-receiver')

        with patch('onboardmimic.backends.sonic.pnlink.reader.read_state', side_effect=receive):
            self.reader.run()
        result = self.reader.snapshot()
        self.assertEqual(len(reads), 10)
        self.assertEqual(result['receiver_id'], 'current-receiver')
        self.assertTrue(result['ready'])
        self.assertEqual(result['raw']['receiver_id'], 'current-receiver')


if __name__=='__main__':unittest.main()
