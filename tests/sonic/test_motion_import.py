"""Conversion contracts, corruption and clock tests without hardware."""
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

import joblib
import numpy as np

from onboardmimic.backends.sonic.motion_import import import_gmr, smpl_arrays, conversion_fingerprint
from onboardmimic.backends.sonic.csv_player import CSVClip, CSVPlayer
from onboardmimic.backends.sonic.smpl_player import SMPLClip
from tests.runtime.test_local_playback import unpack


def native_reference_to_mujoco():
    # Read the actual consumer's inverse permutation, independently of the
    # Python importer's mapping. This is also used by SONIC's CSV visualizer.
    header = Path(__file__).resolve().parents[2]/'onboardmimic/backends/sonic/deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp'
    values = re.search(r'const std::array<int, 29> isaaclab_to_mujoco\s*=\s*\{([^}]+)\}', header.read_text()).group(1)
    return [int(value) for value in values.split(',')]


def write_smpl(path, frames=40, **changes):
    arrays = smpl_arrays(dict(body_pose=np.zeros((frames,63)), smpl_joints=np.zeros((frames,24,3))))
    arrays['body_quat_w'][:] = [1,0,0,0]
    meta = dict(version='onboard-motion-1',fps=50,frames=frames,representation='smpl',conversion_sha256=conversion_fingerprint())
    meta.update(changes)
    np.savez(path, metadata=np.array(json.dumps(meta)), **arrays)


class ImportTest(unittest.TestCase):
    def test_gmr_resampling_order_and_exclusive_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            q = np.tile(np.arange(29),(4,1)).astype(float)
            q += np.arange(4)[:,None]*.01
            data = dict(fps=30,root_pos=np.zeros((4,3)),root_rot=np.tile([0,0,0,1],(4,1)),dof_pos=q)
            joblib.dump(data,root/'in.pkl')
            meta = import_gmr(root/'in.pkl',root/'out','mujoco')
            clip = CSVClip(root/'out',fps=50,joint_order='sonic_reference')
            self.assertEqual(clip.frames,6)
            np.testing.assert_allclose(clip.q[0,native_reference_to_mujoco()],q[0])
            np.testing.assert_allclose(clip.dq,.3,atol=2e-4)
            self.assertEqual(meta['source_duration_seconds'],.1)
            self.assertEqual(meta['options']['joint_mapping'],'gmr_to_sonic_reference_v2')
            with self.assertRaises(ValueError):import_gmr(root/'in.pkl',root/'out','mujoco')
            for field,value in [('fps',float('nan')),('root_rot',np.zeros((4,4))),('dof_pos',np.zeros((3,29)))]:
                bad=dict(data);bad[field]=value;joblib.dump(bad,root/'bad.pkl')
                with self.assertRaises(ValueError):import_gmr(root/'bad.pkl',root/'badout','mujoco')
                self.assertFalse((root/'badout').exists())

    def test_gmr_cli_native_roundtrip_at_30_and_50_hz(self):
        from scipy.spatial.transform import Rotation as R, Slerp
        for fps in (30,50):
            with self.subTest(fps=fps), tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp)
                times=np.arange(31)/fps
                # Distinct trajectories for every joint catch missing, inverse
                # and double permutations through the actual command line.
                q=np.arange(29)[None,:]*.01+times[:,None]*(np.arange(29)[None,:]+1)*.02
                quat=R.from_euler('z',.4+times*.5).as_quat()
                data=dict(fps=fps,dof_pos=q,root_rot=quat,
                          root_pos=np.column_stack((times,np.zeros(31),np.ones(31))))
                joblib.dump(data,root/'in.pkl')
                result=subprocess.run([sys.executable,'-m','onboardmimic.backends.sonic.motion_import',
                                       'gmr',str(root/'in.pkl'),str(root/'out'),
                                       '--input-joint-order','mujoco'],capture_output=True,text=True)
                self.assertEqual(result.returncode,0,result.stderr)
                clip=CSVClip(root/'out',fps=50,joint_order='sonic_reference')
                inverse=native_reference_to_mujoco()
                output_times=np.linspace(0,times[-1],round(times[-1]*50)+1)
                expected=np.arange(29)[None,:]*.01+output_times[:,None]*(np.arange(29)[None,:]+1)*.02
                np.testing.assert_allclose(clip.q[:,inverse],expected,atol=1e-6,rtol=0)
                np.testing.assert_allclose(clip.dq[:,inverse],np.tile((np.arange(29)+1)*.02,(clip.frames,1)),atol=4e-6,rtol=0)
                actual=R.from_quat(clip.quat[:,[1,2,3,0]])
                expected_rotation=Slerp(times,R.from_quat(quat))(output_times)
                self.assertLess(np.max((actual*expected_rotation.inv()).magnitude()),2e-6)
                body=np.loadtxt(root/'out/body_pos.csv',delimiter=',',skiprows=1)
                np.testing.assert_allclose(body[:,0],output_times,atol=1e-6,rtol=0)

    def test_gmr_isaaclab_input_is_already_interleaved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            q=np.tile(np.arange(29)*.01,(4,1))
            joblib.dump(dict(fps=50,dof_pos=q,root_pos=np.zeros((4,3)),
                             root_rot=np.tile([0,0,0,1],(4,1))),root/'in.pkl')
            meta=import_gmr(root/'in.pkl',root/'out','isaaclab')
            clip=CSVClip(root/'out',fps=50,joint_order='sonic_reference')
            np.testing.assert_allclose(clip.q,q,atol=1e-6,rtol=0)
            self.assertEqual(meta['options']['source_column_gather'],list(range(29)))

    def test_smpl_clock_pause_packet_and_stale_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'clip.npz';write_smpl(path)
            clip=SMPLClip(path);player=CSVPlayer();player.load(clip);player.start(1)
            h,d=unpack(player.packet(1));self.assertEqual(h['v'],3)
            self.assertEqual(d['smpl_pose'].shape,(64,21,3));self.assertEqual(d['smpl_joints'].shape,(64,24,3))
            player.confirm(1);player.packet(1.02);player.pause()
            for i in range(100): player.packet(1.04+i*.02)
            self.assertAlmostEqual(player.position,.02)
            player.resume(4);player.packet(4.02)  # producer watchdog rejects a production gap
            self.assertEqual(player.state,'error')
            write_smpl(path,conversion_sha256='stale')
            with self.assertRaises(ValueError):SMPLClip(path)

    def test_file_wrist_slots_match_publisher(self):
        pose=np.zeros((2,21,3));pose[:,19,0]=.2;pose[:,20,0]=.3
        a=smpl_arrays(dict(smpl_pose=pose,smpl_joints=np.zeros((2,24,3))))
        self.assertAlmostEqual(a['joint_pos'][0,23],.2,places=6)
        self.assertAlmostEqual(a['joint_pos'][0,26],-.3,places=6)
        np.testing.assert_array_equal(a['joint_vel'],0)
