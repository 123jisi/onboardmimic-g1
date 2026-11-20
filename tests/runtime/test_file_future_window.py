"""Check native lookahead contract without control, network or hardware."""
from pathlib import Path
import tempfile,unittest
import numpy as np
from onboardmimic.backends.sonic.csv_player import CSVClip,CSVPlayer
from tests.runtime.test_local_playback import write_clip,unpack
from tests.sonic.test_motion_import import write_smpl
from onboardmimic.backends.sonic.smpl_player import SMPLClip

class FutureWindowTest(unittest.TestCase):
    def test_real_future_end_clamp_and_entire_admission_pause_hold(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);write_clip(root/'joint',frames=80);write_smpl(root/'human.npz')
            for clip in (CSVClip(root/'joint',fps=50,joint_order='sonic_reference'),SMPLClip(root/'human.npz')):
                p=CSVPlayer();p.load(clip);p.start(10.)
                data=unpack(p.packet(10.))[1]
                self.assertEqual(data['frame_index'].tolist(),list(range(64)))
                np.testing.assert_array_equal(data['joint_pos'],np.repeat(clip.q[:1],64,axis=0))
                self.assertFalse(data['joint_vel'].any())
                p.confirm(10.)
                data=unpack(p.packet(10.02))[1];idx=np.minimum(np.arange(64)+1,clip.frames-1)
                np.testing.assert_array_equal(data['joint_pos'],clip.q[idx])
                np.testing.assert_array_equal(data['joint_vel'],clip.dq[idx])
                if clip.mode=='smpl_playback':np.testing.assert_array_equal(data['smpl_joints'],clip.arrays['smpl_joints'][idx])
                p.pause();pos=p.position
                for n in range(2,20):data=unpack(p.packet(10.+n*.02))[1]
                self.assertEqual(p.position,pos)
                np.testing.assert_array_equal(data['joint_pos'],np.repeat(clip.q[1:2],64,axis=0))
                self.assertFalse(data['joint_vel'].any())
                p.resume(10.4);p.packet(10.42);self.assertAlmostEqual(p.position,pos+.02)
                # Advance only the test clock to the final source frame, keeping producer age fresh.
                p.position=(clip.frames-1)/50.;p.clock_at=p.produced_at=11.
                data=unpack(p.packet(11.))[1]
                np.testing.assert_array_equal(data['joint_pos'],np.repeat(clip.q[-1:],64,axis=0))
                self.assertIsNone(p.packet(11.02));self.assertEqual(p.state,'finished')
