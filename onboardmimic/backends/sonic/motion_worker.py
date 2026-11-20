"""Isolated catalog/import/load worker. Never imports a transport or robot SDK."""
import json
import os
from pathlib import Path
import sys
import tempfile

import numpy as np

from .motion_library import (scan_catalog, validated_cache, inside, digest, json_file, cache_metadata)


def execute(request, area):
    action = request['action']
    root = Path(request['root'])
    if action == 'scan':
        return dict(entries=scan_catalog(root, request['source_root']))
    if action == 'load':
        spec = request['spec']
        path = inside(root, spec['motion'])
        if spec.get('trusted_cli'):
            from .adapter import SonicBackend
            clip = SonicBackend().load_clip(path, **{k:spec[k] for k in ('fps','joint_order','root_body_index')})
        else:
            clip = validated_cache(root, spec['motion'])
        meta = clip.metadata()
        if spec.get('expected_hashes') and spec['expected_hashes'] != meta.get('hashes'):
            raise ValueError('动作内容已变化，请刷新动作库')
        if spec.get('expected_cache_sha256') and spec['expected_cache_sha256'] != meta.get('cache_sha256'):
            raise ValueError('动作缓存已变化，请刷新动作库')
        arrays = dict(q=clip.q, dq=clip.dq, quat=clip.quat)
        if clip.mode == 'smpl_playback': arrays.update(clip.arrays)
        for name, array in arrays.items(): np.save(area/(name+'.npy'), array, allow_pickle=False)
        return dict(mode=clip.mode, metadata=clip.metadata(), arrays=list(arrays), motion=spec['motion'])
    if action == 'register':
        path = inside(root, request['motion'])
        clip = validated_cache(root, request['motion'], request)
        declaration = dict(fps=50, joint_order='sonic_reference',
                           root_body_index=clip.root_body_index, hashes=clip.hashes)
        stage = path/('.declaration-'+area.name+'.json')
        try:
            stage.write_text(json.dumps(declaration, indent=2))
            os.replace(stage, path/'onboard_csv.json')
        finally: stage.unlink(missing_ok=True)
        return dict(motion=request['motion'], metadata=clip.metadata())
    if action != 'convert': raise ValueError('Unknown worker action')
    from .motion_import import import_gmr, import_pt
    from .kimodo_import import import_kimodo
    path = Path(request['input'])
    if digest(path) != request['source_sha256']: raise ValueError('源文件已变化，请刷新')
    options, kind = request['options'], request['kind']
    # Keep importers' exclusive publication and never expose incomplete output.
    with tempfile.TemporaryDirectory(dir=root, prefix=request.get('staging_prefix','.import-')) as temporary:
        stage = Path(temporary)
        output = stage/('result' if kind == 'gmr' else 'result.npz')
        if kind == 'gmr': meta = import_gmr(path, output, options['joint_order'])
        elif kind == 'smpl': meta = import_pt(path, output, options['fps'], options['group'])
        else: meta = import_kimodo(path, output, options['fps'])
        if meta['source_sha256'] != request['source_sha256'] or digest(path) != request['source_sha256']:
            raise ValueError('转换期间源文件已变化')
        validated_cache(stage, output.name)
        if request.get('publication'):
            # Parent validates, checks cancellation and commits the replacement.
            ready=root/(request['staging_prefix']+'ready'+('' if kind=='gmr' else '.npz'))
            if kind=='gmr':output.rename(ready)
            else:os.link(str(output),str(ready))
            return dict(motion=ready.name,reused=False,metadata=meta)
        key = digest_key(meta)
        for previous in sorted(root.iterdir()):
            if previous.name.startswith('.') or previous.is_symlink():
                continue
            try:
                old = validated_cache(root, previous.name)
                old_meta = cache_metadata(previous, old)
                if digest_key(old_meta) == key:
                    return dict(motion=previous.name, reused=True, metadata=old_meta)
            except (ValueError, KeyError, OSError):
                continue
        name = kind+'_'+path.stem[:50]+'_'+key[:16]+('' if kind == 'gmr' else '.npz')
        target = root/name
        if target.exists():
            old = validated_cache(root, name)
            return dict(motion=name, reused=True, metadata=old.metadata())
        if kind == 'gmr': output.rename(target)
        else: os.link(output, target)
        return dict(motion=name, reused=False, metadata=meta)


def digest_key(meta):
    import hashlib
    identity = {k:meta[k] for k in ('source_sha256','source_format','source_fps','conversion_sha256')}
    options = meta['options']
    # Canonical effective semantics: how a FPS/group was selected is provenance,
    # not a distinct conversion when the selected values are identical.
    identity['options'] = {k:options.get(k) for k in
        ('input_joint_order','joint_mapping','selected_group','wrist_mapping','root_transform')}
    return hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()


if __name__ == '__main__':
    area = Path(sys.argv[1])
    try:
        result = execute(json.loads((area/'request.json').read_text()), area)
        (area/'result.json').write_text(json.dumps(result, ensure_ascii=False))
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)
