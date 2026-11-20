"""Named replacement against real GMR/Kimodo workers; no robot transport."""
from pathlib import Path
import json,tempfile,unittest
from unittest.mock import patch
import joblib
import numpy as np
from onboardmimic.backends.sonic.motion_import import import_gmr
from onboardmimic.backends.sonic.kimodo_import import import_kimodo
from onboardmimic.backends.sonic.motion_library import MotionLibrary,scan_catalog,cache_identity,validated_cache
from onboardmimic.backends.sonic.motion_publication import output_name

class NamedConversionTest(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.base=Path(self.temp.name)
  self.root=self.base/'motions';self.root.mkdir();(self.base/'gmr').mkdir();self.source=self.base/'gmr/wave.pkl'
  joblib.dump(dict(fps=30,dof_pos=np.zeros((4,29)),root_pos=np.zeros((4,3)),root_rot=np.tile([0,0,0,1],(4,1))),self.source)
  import_gmr(self.source,self.root/'old','mujoco');import_gmr(self.source,self.root/'variant','isaaclab')
  self.lib=MotionLibrary(self.root,self.base);self.refresh()
 def refresh(self):self.lib.entries=scan_catalog(self.root,self.base)
 def spec(self,name='新挥手',**options):
  return self.lib.conversion_spec(dict(action='convert',motion_id='gmr/wave.pkl',joint_order='mujoco',output_name=name,**options))
 def visible(self):return sorted(p.name for p in self.root.iterdir() if not p.name.startswith('.'))
 def test_rename_replaces_all_same_source_variants(self):
  other=self.base/'gmr/other.pkl';joblib.dump(dict(fps=30,dof_pos=np.ones((4,29))*.01,root_pos=np.zeros((4,3)),root_rot=np.tile([0,0,0,1],(4,1))),other)
  import_gmr(other,self.root/'other','mujoco');original=cache_identity(self.root/'other');self.refresh()
  result=self.lib.convert(self.spec(),lambda:False)
  self.assertEqual(self.visible(),['other','新挥手']);self.assertEqual(result['replaced_motions'],['old','variant']);self.assertFalse(result['reused'])
  self.assertEqual(cache_identity(self.root/'other'),original);validated_cache(self.root,'新挥手')
  self.assertFalse(list(self.root.glob('.*')));self.assertFalse((self.base/'motion-archive').exists())
 def test_same_name_really_regenerates_and_keeps_only_new(self):
  old_inode=(self.root/'old').stat().st_ino
  result=self.lib.convert(self.spec('old'),lambda:False)
  self.assertFalse(result['reused']);self.assertNotEqual((self.root/'old').stat().st_ino,old_inode);self.assertEqual(self.visible(),['old'])
 def test_unsafe_or_blank_name_is_rejected_without_changes(self):
  for value in ['', '  ','../escape','/tmp/out','a/b','a\\b','.hidden','name.','a\nname','x'*81]:
   with self.subTest(value=value),self.assertRaises(ValueError):self.spec(value)
  self.assertEqual(self.visible(),['old','variant'])
 def test_name_normalization_and_extensions(self):
  self.assertEqual(output_name('  genmo_挥手.npz  ','smpl'),'genmo_挥手.npz')
  self.assertEqual(output_name('kimodo_wave','kimodo'),'kimodo_wave.npz');self.assertEqual(output_name('gmr_wave','gmr'),'gmr_wave')
 def test_collision_with_other_file_is_not_overwritten(self):
  (self.root/'occupied').write_text('unrelated')
  with self.assertRaisesRegex(ValueError,'占用'):self.spec('occupied')
  self.assertEqual((self.root/'occupied').read_text(),'unrelated')
 def test_collision_appearing_during_worker_keeps_old(self):
  spec=self.spec('new');(self.root/'new').write_text('concurrent file')
  with self.assertRaisesRegex(ValueError,'占用'):self.lib.convert(spec,lambda:False)
  self.assertEqual((self.root/'new').read_text(),'concurrent file');self.assertTrue((self.root/'old').is_dir());self.assertTrue((self.root/'variant').is_dir())
 def test_worker_failure_preserves_old(self):
  before=cache_identity(self.root/'old')
  with patch('onboardmimic.backends.sonic.motion_library.run_worker',side_effect=ValueError('worker failed')):
   with self.assertRaisesRegex(ValueError,'worker failed'):self.lib.convert(self.spec(),lambda:False)
  self.assertEqual(cache_identity(self.root/'old'),before);self.assertEqual(self.visible(),['old','variant'])
 def test_cancel_after_generation_keeps_old_and_cleans_stage(self):
  from onboardmimic.backends.sonic import motion_library
  real=motion_library.run_worker;done=[False]
  def worker(*a,**kw):
   answer=real(*a,**kw);done[0]=True;return answer
  with patch.object(motion_library,'run_worker',side_effect=worker):
   with self.assertRaisesRegex(ValueError,'取消'):self.lib.convert(self.spec(),lambda:done[0])
  self.assertEqual(self.visible(),['old','variant']);self.assertFalse(list(self.root.glob('.import-*')))
 def test_changed_old_or_source_prevents_replacement(self):
  spec=self.spec();p=self.root/'old/joint_pos.csv';p.write_text(p.read_text()+'\n')
  with self.assertRaisesRegex(ValueError,'变化'):self.lib.convert(spec,lambda:False)
  self.assertEqual(self.visible(),['old','variant'])
  self.refresh();spec=self.spec();self.source.write_bytes(b'changed')
  with self.assertRaisesRegex(ValueError,'变化'):self.lib.convert(spec,lambda:False)
  self.assertEqual(self.visible(),['old','variant'])
 def test_new_same_source_result_during_conversion_prevents_delete(self):
  spec=self.spec();import_gmr(self.source,self.root/'concurrent','mujoco')
  with self.assertRaisesRegex(ValueError,'同源结果已变化'):self.lib.convert(spec,lambda:False)
  self.assertEqual(self.visible(),['concurrent','old','variant'])
 def test_publish_failure_rolls_back_all_old_names(self):
  real=Path.rename;before={n:cache_identity(self.root/n) for n in ['old','variant']}
  def fail_new(path,target):
   if path.name.startswith('.import-') and str(target).endswith('/new'):raise OSError('publish failed')
   return real(path,target)
  with patch.object(Path,'rename',fail_new):
   with self.assertRaisesRegex(OSError,'publish failed'):self.lib.convert(self.spec('new'),lambda:False)
  self.assertEqual(self.visible(),['old','variant']);self.assertEqual({n:cache_identity(self.root/n) for n in before},before)
  self.assertFalse(list(self.root.glob('.*')))
 def test_kimodo_npz_uses_same_transaction_and_removes_old_file(self):
  (self.base/'kimodo').mkdir();source=self.base/'kimodo/wave.npz'
  np.savez(source,pose_body=np.zeros((4,63)),root_orient=np.zeros((4,3)),mocap_frame_rate=np.array(30.))
  import_kimodo(source,self.root/'kimodo_old.npz');self.refresh()
  spec=self.lib.conversion_spec(dict(action='convert',motion_id='kimodo/wave.npz',output_name='kimodo_新动作',fps=None))
  result=self.lib.convert(spec,lambda:False);self.assertEqual(result['motion'],'kimodo_新动作.npz')
  self.assertFalse((self.root/'kimodo_old.npz').exists());validated_cache(self.root,result['motion'])
  self.assertTrue((self.root/'old').exists());self.assertTrue((self.root/'variant').exists());self.assertFalse(list(self.root.glob('.*')))
