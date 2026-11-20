"""AMASS validation, time base, wrist convention and lightweight playback."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation as R

from onboardmimic.backends.sonic.kimodo_import import import_kimodo
from onboardmimic.backends.sonic.motion_import import smpl_arrays
from onboardmimic.backends.sonic.smpl_player import SMPLClip
from onboardmimic.backends.sonic.csv_player import CSVPlayer
from tests.runtime.test_local_playback import unpack


def amass(path, frames=31, **changes):
    pose=np.zeros((frames,63),dtype=np.float32)
    pose[:,19*3:19*3+3]=[.2,.1,.05]
    values=dict(pose_body=pose,root_orient=np.zeros((frames,3)),mocap_frame_rate=np.array(30.))
    values.update(changes)
    np.savez(path,**values)


class KimodoImportTest(unittest.TestCase):
    def test_import_resampling_metadata_no_torch_and_original_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);amass(root/'amass.npz',gender=np.array('neutral'))
            meta=import_kimodo(root/'amass.npz',root/'cache.npz')
            clip=SMPLClip(root/'cache.npz')
            self.assertEqual(clip.frames,51)
            self.assertEqual(meta['source_fps'],30)
            self.assertEqual(meta['options']['fps_source'],'mocap_frame_rate')
            self.assertAlmostEqual(clip.duration,1.02)
            with np.load(root/'cache.npz') as z:
                self.assertEqual(z['source_smpl_pose'].shape,(31,21,3))
                np.testing.assert_allclose(clip.q[:,25],-.1,atol=.01)
            result=subprocess.run([sys.executable,'-c',
                'import sys; from onboardmimic.backends.sonic.kimodo_import import import_kimodo; '
                'from onboardmimic.backends.sonic.smpl_player import SMPLClip; '
                'import_kimodo(sys.argv[1],sys.argv[2]); SMPLClip(sys.argv[2]); '
                'assert "torch" not in sys.modules and "gear_sonic" not in sys.modules',
                str(root/'amass.npz'),str(root/'other.npz')],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            with self.assertRaises(ValueError):import_kimodo(root/'amass.npz',root/'cache.npz')

    def test_bad_inputs_and_explicit_fps(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for change in (dict(pose_body=np.zeros((31,62))),dict(root_orient=np.zeros((30,3))),
                           dict(root_orient=np.full((31,3),np.nan)),dict(mocap_frame_rate=np.nan),
                           dict(mocap_frame_rate=0),dict(mocap_frame_rate=np.array([30,30])),
                           dict(pose_body=np.array([object()],dtype=object))):
                amass(root/'bad.npz',**change)
                with self.assertRaises(ValueError):import_kimodo(root/'bad.npz',root/'out.npz')
                self.assertFalse((root/'out.npz').exists())
            np.savez(root/'missing.npz',pose_body=np.zeros((4,63)),root_orient=np.zeros((4,3)))
            with self.assertRaisesRegex(ValueError,'--fps'):import_kimodo(root/'missing.npz',root/'out.npz')
            meta=import_kimodo(root/'missing.npz',root/'out.npz',30)
            self.assertEqual(meta['options']['fps_source'],'explicit')
            np.savez(root/'raw.npz',local_rot_mats=np.zeros((4,22,3,3)))
            with self.assertRaisesRegex(ValueError,'AMASS'):import_kimodo(root/'raw.npz',root/'rawout.npz',30)

    def test_kimodo_wrist_slots_and_sign_are_distinct_from_genmo(self):
        pose=np.zeros((2,21,3));pose[:,19]=[.2,.1,0];pose[:,20]=[.3,.15,0]
        data=dict(smpl_pose=pose,smpl_joints=np.zeros((2,24,3)))
        k=smpl_arrays(data,wrist_mapping='kimodo');g=smpl_arrays(data)
        np.testing.assert_allclose(k['joint_pos'][:,[23,25,27]],g['joint_pos'][:,23:26]*[1,-1,1])
        np.testing.assert_allclose(k['joint_pos'][:,[24,26,28]],g['joint_pos'][:,26:29])
        np.testing.assert_array_equal(k['joint_vel'],0)

    def test_slerp_rotation_and_player_v3_timeline(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            orient=np.zeros((31,3));orient[:,2]=np.linspace(0,1,31)
            amass(root/'amass.npz',root_orient=orient)
            import_kimodo(root/'amass.npz',root/'cache.npz')
            clip=SMPLClip(root/'cache.npz')
            # Endpoint root rotation is preserved and adjacent quaternions remain continuous.
            self.assertTrue(np.all(np.sum(clip.quat[1:]*clip.quat[:-1],axis=1)>0))
            relative=R.from_quat(clip.quat[-1,[1,2,3,0]])*R.from_quat(clip.quat[0,[1,2,3,0]]).inv()
            self.assertAlmostEqual(relative.magnitude(),1.,places=6)
            player=CSVPlayer();player.load(clip);player.start(1)
            h,d=unpack(player.packet(1));self.assertEqual(h['v'],3)
            player.confirm(1)
            for i in range(1,52):player.packet(1+i*.02)
            self.assertEqual(player.state,'finished')
            self.assertAlmostEqual(player.position,1.02)
