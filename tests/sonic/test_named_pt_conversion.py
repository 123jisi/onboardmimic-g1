"""Actual CPU PT worker naming and replacement, no robot transport."""
import importlib.util
import tempfile,unittest
from pathlib import Path
from onboardmimic.backends.sonic.motion_library import MotionLibrary,scan_catalog,validated_cache
@unittest.skipUnless(importlib.util.find_spec('torch'),'PT import environment needs Torch')
class NamedPTTest(unittest.TestCase):
 def test_named_pt_reconversion_changes_fps_and_removes_previous(self):
  import torch
  with tempfile.TemporaryDirectory() as temp:
   base=Path(temp);root=base/'motions';root.mkdir();(base/'smpl').mkdir()
   torch.save(dict(body_pose=torch.zeros(4,63),global_orient=torch.zeros(4,3)),base/'smpl/wave.pt')
   lib=MotionLibrary(root,base)
   def convert(name,fps):
    lib.entries=scan_catalog(root,base)
    return lib.convert(lib.conversion_spec(dict(action='convert',motion_id='smpl/wave.pt',output_name=name,fps=fps)),lambda:False)
   self.assertEqual(convert('genmo_wave',50)['motion'],'genmo_wave.npz')
   result=convert('GENMO_新挥手.npz',30)
   self.assertFalse(result['reused']);self.assertEqual(result['replaced_motions'],['genmo_wave.npz']);self.assertEqual(result['metadata']['source_fps'],30)
   self.assertFalse((root/'genmo_wave.npz').exists());validated_cache(root,result['motion'])
   self.assertEqual([p.name for p in root.iterdir()],['GENMO_新挥手.npz'])
