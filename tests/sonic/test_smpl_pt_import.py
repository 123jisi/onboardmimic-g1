"""Actual Torch serialization -> CPU import -> runtime cache and v3 packet."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from onboardmimic.backends.sonic.motion_import import import_pt
from onboardmimic.backends.sonic.smpl_player import SMPLClip
from onboardmimic.backends.sonic.csv_player import CSVPlayer


@unittest.skipUnless(importlib.util.find_spec('torch'), 'PT import environment needs Torch')
class PTImportTest(unittest.TestCase):
    def setUp(self):
        import torch
        self.torch = torch
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root/'smpl_params.pt'
        self.params = dict(body_pose=torch.zeros(61, 63), global_orient=torch.zeros(61, 3))
        self.params['body_pose'][:, 57] = torch.linspace(0, .3, 61)

    def test_cli_nested_pt_resampling_and_runtime_packet(self):
        self.torch.save(dict(body_params_global=self.params), self.source)
        out = self.root/'motion.npz'
        r = subprocess.run([sys.executable, '-m', 'onboardmimic.backends.sonic.motion_import',
                            'smpl', str(self.source), str(out), '--fps', '30',
                            '--group', 'body_params_global'], capture_output=True, text=True, check=True)
        meta = json.loads(r.stdout)
        self.assertEqual((meta['source_frames'], meta['frames'], meta['fps']), (61, 101, 50))
        self.assertEqual(meta['options']['selected_group'], 'body_params_global')
        clip = SMPLClip(out)
        np.testing.assert_allclose(clip.arrays['smpl_pose'][[0, -1], 19, 0], [0, .3], atol=1e-6)
        with np.load(out, allow_pickle=False) as z:
            np.testing.assert_array_equal(z['source_smpl_pose'], self.params['body_pose'].numpy().reshape(61,21,3))
        player = CSVPlayer(); player.load(clip); player.start(1.)
        packet = player.packet(1.)
        # Same bytes consumed by the existing local playback mux/native reader.
        header = json.loads(packet[len(b'pose'):len(b'pose')+1280].rstrip(b'\x00'))
        self.assertEqual(header['v'], 3)
        self.assertEqual(clip.mode, 'smpl_playback')

    def test_top_level_auto_global_precedence_and_exclusive_output(self):
        for name, raw, group, selected in (
            ('flat', self.params, 'auto', None),
            ('global', dict(body_params_global=self.params, body_params_incam={}), 'auto', 'body_params_global'),
            ('incam', dict(body_params_incam=self.params), 'body_params_incam', 'body_params_incam'),
        ):
            with self.subTest(name=name):
                self.torch.save(raw, self.source)
                out = self.root/(name+'.npz')
                meta = import_pt(self.source, out, 50, group)
                self.assertEqual(meta['options']['selected_group'], selected)
                self.assertEqual(SMPLClip(out).frames, 61)
                before = out.read_bytes()
                with self.assertRaisesRegex(ValueError, 'already exists'):
                    import_pt(self.source, out, 50, group)
                self.assertEqual(out.read_bytes(), before)

    def test_invalid_pt_does_not_publish_cache(self):
        bad = dict(self.params)
        bad['body_pose'] = self.params['body_pose'].clone()
        bad['body_pose'][0, 0] = float('nan')
        for raw in (bad, dict(body_params_global=42), [1, 2]):
            with self.subTest(layout=type(raw).__name__):
                self.torch.save(raw, self.source)
                out = self.root/'bad.npz'
                with self.assertRaises((ValueError, TypeError)):
                    import_pt(self.source, out, 50, 'auto')
                self.assertFalse(out.exists())


if __name__ == '__main__':
    unittest.main()
