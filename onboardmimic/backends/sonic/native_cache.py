"""Offline audit of the cache files actually read by the native SONIC loader.

Raw onboard/engines/*.engine files alone do not satisfy the native cache lookup.
This module never imports CUDA/TensorRT, constructs a controller, or builds plans.
"""
import hashlib
from pathlib import Path

CACHE_SPECS = (
    ('encoder', 'policy/release/model_encoder.onnx', 'policy/release/encoder_model_encoder.trt'),
    ('decoder', 'policy/release/model_decoder.onnx', 'policy/release/policy_model_decoder.trt'),
    ('planner', 'planner/target_vel/V2/planner_orin_compatible.onnx',
     'planner/target_vel/V2/planner_planner_orin_compatible.trt'),
)


def digest(path, skip=0):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        stream.seek(skip)
        for block in iter(lambda: stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def verify_native_caches(deploy, gpu='Orin'):
    """Verify FP32 cache key and exact payload identity with validated engines.

    Matches InferenceEngine.cpp: SHA256(SHA256(SHA256(ONNX)+GPU)+'0').
    The 64 ASCII key bytes prefix the unchanged raw TensorRT engine payload.
    Run offline before commissioning; whole-file hashing is not a control tick.
    """
    deploy = Path(deploy)
    records = []
    for name, model_name, cache_name in CACHE_SPECS:
        model, cache = deploy/model_name, deploy/cache_name
        engine = deploy/'onboard/engines'/(name+'_fp32.engine')
        for path in (model, engine, cache):
            if not path.is_file():
                raise ValueError('Missing native inference asset: '+str(path))
        model_digest = digest(model)
        expected = hashlib.sha256((model_digest+gpu).encode()).hexdigest()
        expected = hashlib.sha256((expected+'0').encode()).hexdigest()
        with cache.open('rb') as stream:
            header = stream.read(64)
        if header != expected.encode('ascii'):
            raise ValueError('Native cache key mismatch (model/GPU/FP32): '+cache_name)
        engine_digest = digest(engine)
        if cache.stat().st_size != engine.stat().st_size+64 or digest(cache, 64) != engine_digest:
            raise ValueError('Native cache payload differs from validated raw engine: '+cache_name)
        records.append(dict(model=model_name, cache=cache_name, gpu=gpu, precision='FP32',
                            cache_key=expected, cache_sha256=digest(cache),
                            model_sha256=model_digest, engine_sha256=engine_digest))
    return records
