"""Reconvert stale caches by source identity; archive only after valid success."""
from pathlib import Path
import tempfile,json,unittest
from unittest.mock import patch,Mock
import numpy as np
import joblib
from onboardmimic.backends.sonic.motion_import import import_gmr
from onboardmimic.backends.sonic.motion_library import MotionLibrary,scan_catalog,validated_cache,digest,cache_identity

class MotionReimportTest(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.base=Path(self.tmp.name)
  self.root=self.base/'motions';self.root.mkdir();(self.base/'gmr').mkdir();self.source=self.base/'gmr/wave.pkl'
  joblib.dump(dict(fps=30,dof_pos=np.zeros((4,29)),root_pos=np.zeros((4,3)),root_rot=np.tile([0,0,0,1],(4,1))),self.source)
  self.old=self.root/'old';import_gmr(self.source,self.old,'mujoco')
  p=self.old/'import.json';m=json.loads(p.read_text());m['options'].pop('joint_mapping');p.write_text(json.dumps(m))
  self.lib=MotionLibrary(self.root,self.base);self.refresh()
 def refresh(self):self.lib.entries=scan_catalog(self.root,self.base)
 def spec(self):return self.lib.conversion_spec(dict(action='convert',motion_id='cache/old',joint_order='mujoco'))
 def test_rebuild_validates_new_then_archives_old(self):
  result=self.lib.convert(self.spec(),lambda:False)
  self.assertFalse(self.old.exists());self.assertTrue(Path(result['archive']).is_dir())
  validated_cache(self.root,result['motion']);self.assertEqual(result['retired_motion'],'old')
 def test_matching_good_cache_is_reused_and_preserved(self):
  import_gmr(self.source,self.root/'verified','mujoco');before=cache_identity(self.root/'verified');self.refresh()
  result=self.lib.convert(self.spec(),lambda:False)
  self.assertTrue(result['reused']);self.assertEqual(result['motion'],'verified')
  self.assertEqual(cache_identity(self.root/'verified'),before);self.assertFalse(self.old.exists())
 def test_source_missing_or_changed_never_retires_old(self):
  self.source.unlink();self.refresh()
  with self.assertRaisesRegex(ValueError,'缺少匹配'):self.spec()
  self.assertTrue(self.old.exists())
 def test_failure_or_cancel_keeps_old(self):
  spec=self.spec()
  with patch('onboardmimic.backends.sonic.motion_library.run_worker',side_effect=ValueError('failed')):
   with self.assertRaisesRegex(ValueError,'failed'):self.lib.convert(spec,lambda:False)
  self.assertTrue(self.old.exists());self.assertFalse((self.base/'motion-archive').exists())
 def test_cache_change_during_reconversion_is_not_retired(self):
  spec=self.spec();p=self.old/'joint_pos.csv';p.write_text(p.read_text()+'\n')
  with self.assertRaisesRegex(ValueError,'旧缓存内容已变化'):self.lib.convert(spec,lambda:False)
  self.assertTrue(self.old.exists());self.assertFalse((self.base/'motion-archive').exists())
 def test_symlink_cache_content_is_refused(self):
  (self.old/'link').symlink_to(self.source)
  with self.assertRaisesRegex(ValueError,'普通文件'):self.spec()

 def test_cancel_after_worker_success_keeps_old(self):
  spec=self.spec();area=Mock()
  with patch('onboardmimic.backends.sonic.motion_library.run_worker',return_value=({'motion':'new'},area)):
   with self.assertRaisesRegex(ValueError,'任务已取消'):self.lib.convert(spec,lambda:True)
  area.cleanup.assert_called_once();self.assertTrue(self.old.exists())
