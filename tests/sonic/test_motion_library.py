import json
from pathlib import Path
import tempfile
import unittest
import zipfile

import numpy as np

from onboardmimic.backends.sonic.motion_library import (MotionLibrary, scan_catalog,
    validated_cache, inside, run_worker, digest)
from onboardmimic.backends.sonic.motion_worker import execute
from tests.runtime.test_local_playback import write_clip


class MotionLibraryTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.base=Path(self.temp.name);self.root=self.base/'motions';self.root.mkdir()

    def register(self,name='csv'):
        return execute(dict(action='register',root=str(self.root),motion=name,fps=50,
            joint_order='sonic_reference',root_body_index=0),self.base)

    def test_missing_sources_and_csv_declaration_hash(self):
        write_clip(self.root/'csv')
        rows=scan_catalog(self.root,self.base)
        self.assertEqual(rows[0]['status'],'needs_confirmation')
        self.register()
        self.assertEqual(scan_catalog(self.root,self.base)[0]['status'],'ready')
        p=self.root/'csv/joint_pos.csv';p.write_text(p.read_text().replace('0.000000000000000000e+00','1.000000000000000000e-04'))
        with self.assertRaisesRegex(ValueError,'变化'):validated_cache(self.root,'csv')

    def test_known_bad_gmr_cannot_be_manually_registered(self):
        write_clip(self.root/'csv')
        (self.root/'csv/import.json').write_text(json.dumps(dict(source_format='gmr_pkl', options={})))
        with self.assertRaisesRegex(ValueError,'映射'):self.register()
        self.assertEqual(scan_catalog(self.root,self.base)[0]['status'],'needs_reimport')

    def test_path_traversal_and_symlink(self):
        outside=self.base/'outside';outside.mkdir();(self.root/'link').symlink_to(outside)
        for name in ('../outside','link','/tmp'):
            with self.subTest(name=name),self.assertRaises(ValueError):inside(self.root,name)
        self.assertEqual(scan_catalog(self.root,self.base),[])

    def test_raw_files_are_not_deserialized_by_listing(self):
        (self.base/'gmr').mkdir();(self.base/'gmr/a.pkl').write_bytes(b'not a pickle')
        rows=scan_catalog(self.root,self.base)
        self.assertEqual(rows[0]['status'],'needs_conversion')
        library=MotionLibrary(self.root);library.entries=rows
        with self.assertRaisesRegex(ValueError,'关节顺序'):library.conversion_spec(dict(motion_id='gmr/a.pkl'))

    def test_pt_requires_fps_and_changed_source_is_rejected(self):
        (self.base/'smpl').mkdir();path=self.base/'smpl/a.pt';path.write_bytes(b'not tensors')
        library=MotionLibrary(self.root);library.entries=scan_catalog(self.root,self.base)
        with self.assertRaisesRegex(ValueError,'帧率'):library.conversion_spec(dict(motion_id='smpl/a.pt'))
        spec=library.conversion_spec(dict(motion_id='smpl/a.pt',fps=30))
        self.assertEqual(spec['options']['fps'],30)
        path.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'变化'):library.conversion_spec(dict(motion_id='smpl/a.pt',fps=30))

    def test_gmr_worker_atomic_output_and_reuse(self):
        import joblib
        p=self.base/'a.pkl';n=4
        joblib.dump(dict(fps=30,dof_pos=np.zeros((n,29)),root_pos=np.zeros((n,3)),
            root_rot=np.tile([0,0,0,1],(n,1))),p)
        request=dict(action='convert',kind='gmr',input=str(p),root=str(self.root),
            source_sha256=digest(p),options=dict(joint_order='mujoco'))
        result=execute(request,self.base)
        self.assertFalse(result['reused']);self.assertTrue(execute(request,self.base)['reused'])
        self.assertEqual(scan_catalog(self.root,self.base)[0]['status'],'ready')
        self.assertEqual(len(list(self.root.iterdir())),1)

    def test_worker_cancel_timeout_cleanup(self):
        with self.assertRaisesRegex(ValueError,'取消'):
            run_worker(dict(action='scan',root=str(self.root),source_root=str(self.base)),cancelled=lambda:True)
        with self.assertRaisesRegex(ValueError,'超时'):
            run_worker(dict(action='scan',root=str(self.root),source_root=str(self.base)),timeout=-1)

    def test_raw_amass_is_not_a_playable_cache(self):
        np.savez(self.root/'raw.npz',pose_body=np.zeros((4,63)),root_orient=np.zeros((4,3)),fps=30)
        self.assertEqual(scan_catalog(self.root,self.base)[0]['status'],'invalid')

    def test_csv_registration_requires_explicit_order_and_rate(self):
        write_clip(self.root/'csv')
        for options in ({'fps':30,'joint_order':'sonic_reference'},{'fps':50,'joint_order':'mujoco'}):
            with self.subTest(options=options),self.assertRaises(ValueError):
                execute(dict(action='register',root=str(self.root),motion='csv',**options),self.base)

    def test_cancelled_worker_staging_is_removed_by_parent(self):
        from unittest.mock import patch
        library=MotionLibrary(self.root)
        def killed(spec, **kwargs):
            stage=self.root/(spec['staging_prefix']+'partial');stage.mkdir()
            (stage/'partial.csv').write_text('incomplete')
            raise ValueError('任务已取消')
        with patch('onboardmimic.backends.sonic.motion_library.run_worker',side_effect=killed):
            with self.assertRaisesRegex(ValueError,'取消'):
                library.convert(dict(action='convert',kind='gmr'),lambda:True)
        self.assertEqual(list(self.root.iterdir()),[])

    def test_changed_cache_after_catalog_is_not_loaded(self):
        from tests.sonic.test_motion_import import write_smpl
        from onboardmimic.backends.sonic.smpl_player import SMPLClip
        p=self.root/'human.npz';write_smpl(p)
        meta=SMPLClip(p).metadata()
        library=MotionLibrary(self.root)
        library.entries=[dict(id='cache/human.npz',kind='cache',status='ready',motion='human.npz',metadata=meta)]
        spec=library.load_spec(dict(motion_id='cache/human.npz'))
        with np.load(p,allow_pickle=False) as data:
            arrays={k:np.array(data[k]) for k in data.files}
        arrays['joint_pos'][0,0]=.05
        np.savez(p,**arrays)
        with self.assertRaisesRegex(ValueError,'变化'):
            execute(dict(action='load',root=str(self.root),spec=spec),self.base)
