"""Catch missing native wrappers despite present raw TensorRT engine files."""
import hashlib
from pathlib import Path
import tempfile
import unittest

from onboardmimic.backends.sonic.native_cache import CACHE_SPECS, verify_native_caches


class NativeCacheTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        for name, model, cache in CACHE_SPECS:
            path = self.root/model;path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(('model-'+name).encode())
            engine = self.root/'onboard/engines'/(name+'_fp32.engine')
            engine.parent.mkdir(parents=True, exist_ok=True)
            payload = ('validated-plan-'+name).encode();engine.write_bytes(payload)
            key = hashlib.sha256(path.read_bytes()).hexdigest()
            key = hashlib.sha256((key+'Orin').encode()).hexdigest()
            key = hashlib.sha256((key+'0').encode()).hexdigest()
            (self.root/cache).write_bytes(key.encode()+payload)

    def test_all_native_wrappers_match_raw_engine_and_key(self):
        records = verify_native_caches(self.root)
        self.assertEqual(len(records), 3)
        self.assertEqual(records[2]['cache'], CACHE_SPECS[2][2])

    def test_raw_engines_without_native_cache_do_not_pass(self):
        cache = self.root/CACHE_SPECS[2][2];cache.unlink()
        with self.assertRaisesRegex(ValueError, 'Missing native inference asset'):
            verify_native_caches(self.root)
        self.assertFalse(cache.exists())

    def test_changed_onnx_or_gpu_invalidates_existing_cache(self):
        with self.assertRaisesRegex(ValueError, 'key mismatch'):
            verify_native_caches(self.root, gpu='Different GPU')
        (self.root/CACHE_SPECS[0][1]).write_bytes(b'new-model')
        with self.assertRaisesRegex(ValueError, 'key mismatch'):
            verify_native_caches(self.root)

    def test_rebuilt_or_truncated_payload_is_not_trusted(self):
        cache = self.root/CACHE_SPECS[0][2]
        cache.write_bytes(cache.read_bytes()[:64]+b'unqualified-rebuild')
        with self.assertRaisesRegex(ValueError, 'payload differs'):
            verify_native_caches(self.root)


if __name__ == '__main__':unittest.main()
