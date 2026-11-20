"""Bounded local catalog and worker supervision. No controller or control socket."""
import hashlib
import json
import os
import secrets
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time

from onboardmimic.core.paths import ROOT


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def cache_identity(path):
    """Snapshot a stale flat cache before archiving; never follow cache symlinks."""
    path=Path(path)
    if path.is_symlink():raise ValueError('旧缓存不能为符号链接')
    if path.is_file():return {'':digest(path)}
    children=sorted(path.iterdir())
    if any(p.is_symlink() or not p.is_file() for p in children):
        raise ValueError('旧缓存必须是普通文件组成的目录')
    return {p.name:digest(p) for p in children}


def inside(root, name):
    root = Path(root).resolve()
    if not isinstance(name, str) or not name or Path(name).is_absolute():
        raise ValueError('需要根目录内的相对路径')
    p = (root/name).resolve(strict=True)
    if p == root or root not in p.parents:
        raise ValueError('动作路径越界')
    return p


def json_file(path):
    if path.is_symlink():
        raise ValueError('元数据不能为符号链接')
    if path.stat().st_size > 65536:
        raise ValueError('元数据过大')
    return json.loads(path.read_text())


def csv_provenance(path):
    """Known bad conversions cannot be admitted by the manual CSV declaration."""
    from .csv_player import REFERENCE_TO_HARDWARE
    from .motion_import import conversion_fingerprint
    meta = json_file(path/'import.json') if (path/'import.json').exists() else None
    if meta is not None:
        if meta.get('source_format') != 'gmr_pkl':
            raise ValueError('未知 CSV 转换来源')
        options = meta.get('options', {})
        expected = list(REFERENCE_TO_HARDWARE) if options.get('input_joint_order') == 'mujoco' else list(range(29))
        if options.get('joint_mapping') != 'gmr_to_sonic_reference_v2' or options.get('source_column_gather') != expected:
            raise ValueError('旧版或未知 GMR 关节映射，必须重新转换')
        if meta.get('conversion_sha256') != conversion_fingerprint():
            raise ValueError('转换版本已变化，请重新转换')
        return meta
    info = path/'info.txt'
    if info.is_symlink():
        raise ValueError('元数据不能为符号链接')
    if info.exists():
        with info.open() as stream:
            if 'mujoco' in stream.read(65536).lower():
                raise ValueError('旧 CSV 声明 MuJoCo 顺序，必须重新转换')
    return None


def validated_cache(root, name, declaration=None):
    from .csv_player import CSVClip
    from .smpl_player import SMPLClip
    p = inside(root, name)
    if p.is_file() and p.suffix == '.npz':
        return SMPLClip(p)
    if not p.is_dir():
        raise ValueError('需要 CSV 目录或 SMPL 缓存')
    meta = csv_provenance(p)
    registration = p/'onboard_csv.json'
    options = declaration
    if meta:
        options = dict(fps=50, joint_order='sonic_reference', root_body_index=0)
    elif options is None and registration.exists():
        options = json_file(registration)
    if options is None:
        raise ValueError('CSV 参数待确认')
    clip = CSVClip(p, fps=options.get('fps'), joint_order=options.get('joint_order'),
                   root_body_index=options.get('root_body_index', 0))
    if not meta and declaration is None and options.get('hashes') != clip.hashes:
        raise ValueError('CSV 内容已变化，请重新登记')
    return clip


def cache_metadata(path, clip):
    meta = clip.metadata()
    if path.is_dir() and (path/'import.json').exists():
        meta = dict(json_file(path/'import.json'), **meta)
    return meta


def scan_catalog(root, source_root):
    root, source_root = Path(root).resolve(), Path(source_root).resolve()
    entries = []
    # Only bounded, explicit two-level layouts; never recurse through symlinks.
    candidates = sorted(p for p in root.iterdir() if not p.name.startswith('.') and not p.is_symlink())[:1000]
    for p in candidates:
        if not (p.is_dir() or p.suffix == '.npz'):
            continue
        row = dict(id='cache/'+p.name, name=p.name, motion=p.name, kind='cache', status='invalid')
        try:
            clip = validated_cache(root, p.name)
            row.update(status='ready', metadata=cache_metadata(p, clip))
        except Exception as exc:
            row['error'] = str(exc)
            if p.is_dir():
                row['status'] = 'needs_confirmation' if '待确认' in str(exc) or '登记' in str(exc) else 'needs_reimport'
                if (p/'import.json').exists():
                    try: row['metadata'] = json_file(p/'import.json')
                    except Exception: pass
            elif 'Stale' in str(exc):
                row['status'] = 'needs_reimport'
                try:
                    import numpy as np
                    import zipfile
                    with zipfile.ZipFile(p) as z:
                        if sum(i.file_size for i in z.infolist()) > 256*1024*1024: raise ValueError('archive size')
                    with np.load(p, allow_pickle=False) as data:
                        row['metadata'] = json.loads(str(data['metadata']))
                except Exception: pass
        entries.append(row)
    for kind, suffix in (('gmr','.pkl'), ('smpl','.pt'), ('kimodo','.npz')):
        directory = source_root/kind
        if not directory.is_dir() or directory.is_symlink():
            continue
        for p in sorted(directory.iterdir())[:1000]:
            if p.is_symlink() or not p.is_file() or p.suffix != suffix:
                continue
            row = dict(id=kind+'/'+p.name, name=p.name, kind=kind, status='needs_conversion', variants=[])
            try:
                inside(source_root, kind+'/'+p.name)
                if p.stat().st_size > 128*1024*1024: raise ValueError('Input exceeds 128 MiB')
                row['source_sha256'] = digest(p)
                row['variants'] = [e['id'] for e in entries if e.get('metadata',{}).get('source_sha256') == row['source_sha256']]
                if any(e['status'] == 'ready' and e['id'] in row['variants'] for e in entries):
                    row['status'] = 'ready'
                elif row['variants']:
                    row['status'] = 'needs_reimport'
            except Exception as exc:
                row.update(status='invalid', error=str(exc))
            entries.append(row)
    return entries


def run_worker(request, cancelled=lambda: False, timeout=60, interpreter=None):
    """One bounded subprocess. Temporary artifacts live as long as their clip."""
    area = tempfile.TemporaryDirectory(prefix='onboard-motion-')
    directory = Path(area.name)
    (directory/'request.json').write_text(json.dumps(request, allow_nan=False))
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
               PYTHONPATH=str(ROOT))
    try:
        with (directory/'output.log').open('w+') as output:
            process = subprocess.Popen([str(interpreter or sys.executable), str(ROOT/'onboardmimic/core/owned_exec.py'),
                str(os.getpid()), str(interpreter or sys.executable), '-m',
                'onboardmimic.backends.sonic.motion_worker', str(directory)],
                cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True)
            deadline = time.monotonic()+timeout
            try:
                while process.poll() is None:
                    if cancelled(): raise ValueError('任务已取消')
                    if time.monotonic() > deadline: raise ValueError('任务执行超时')
                    time.sleep(.025)
                if process.returncode:
                    output.seek(0)
                    raise ValueError(output.read()[-3000:] or '动作处理失败')
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
        result = json.loads((directory/'result.json').read_text())
        return result, area
    except BaseException:
        area.cleanup()
        raise


class MotionLibrary:
    def __init__(self, root, source_root=None):
        self.root = Path(root).resolve()
        self.source_root = Path(source_root or self.root.parent).resolve()
        self.entries = []
        self.state, self.error = 'idle', ''
        self.lock = threading.RLock()

    def snapshot(self):
        with self.lock:
            return dict(state=self.state, error=self.error, entries=list(self.entries))

    def refresh(self):
        with self.lock:
            if self.state == 'scanning': return
            self.state, self.error = 'scanning', ''
        def work():
            try:
                result, area = run_worker(dict(action='scan', root=str(self.root), source_root=str(self.source_root)), timeout=120)
                area.cleanup()
                with self.lock: self.entries, self.state = result['entries'], 'ready'
            except Exception as exc:
                with self.lock: self.state, self.error = 'error', str(exc)
        threading.Thread(target=work, daemon=True, name='motion-catalog').start()

    def entry(self, identity):
        if not isinstance(identity, str): raise ValueError('缺少动作标识')
        with self.lock:
            row = next((e for e in self.entries if e['id'] == identity), None)
            if row is None: raise ValueError('动作未收录，请刷新动作库')
            return dict(row)

    def load_spec(self, x):
        if x.get('motion_id'):
            row = self.entry(x['motion_id'])
            if row['kind'] != 'cache' or row['status'] != 'ready':
                raise ValueError('请先转换或登记可播放动作')
            meta = row.get('metadata', {})
            return dict(motion=row['motion'], expected_hashes=meta.get('hashes'),
                        expected_cache_sha256=meta.get('cache_sha256'))
        # Trusted loopback CLI retains explicit relative-path support.
        name = x.get('motion')
        inside(self.root, name)
        return dict(motion=name, fps=x.get('fps'), joint_order=x.get('joint_order'),
                    root_body_index=x.get('root_body_index',0), trusted_cli=True)

    def conversion_spec(self, x):
        row = self.entry(x.get('motion_id'))
        action = x.get('action')
        retirement=None
        if row['kind']=='cache' and row['status']=='needs_reimport' and action=='convert':
            meta=row.get('metadata',{})
            kind={'gmr_pkl':'gmr','smpl_pt':'smpl','kimodo_amass_npz':'kimodo'}.get(meta.get('source_format'))
            source=next((e for e in self.entries if e.get('kind')==kind and
                         e.get('source_sha256')==meta.get('source_sha256') and e.get('status')!='invalid'),None)
            if not kind or not meta.get('source_sha256') or not source:
                raise ValueError('缺少匹配的原始文件，请上传原文件后刷新动作库')
            path=inside(self.root,row['motion'])
            retirement=dict(motion=row['motion'],identity=cache_identity(path))
            row=source
        if action == 'register':
            if row['kind'] != 'cache' or row['status'] != 'needs_confirmation':
                raise ValueError('此动作不能手动登记')
            return dict(action='register', root=str(self.root), motion=row['motion'],
                        fps=x.get('fps'), joint_order=x.get('joint_order'),
                        root_body_index=x.get('root_body_index',0))
        if row['kind'] not in ('gmr','smpl','kimodo') or row['status'] == 'invalid':
            raise ValueError('请选择原始动作文件')
        path = inside(self.source_root, row['id'])
        if digest(path) != row['source_sha256']:
            raise ValueError('源文件已变化，请刷新')
        import math
        options = {}
        if row['kind'] == 'gmr':
            order = x.get('joint_order')
            if order not in ('mujoco','isaaclab'): raise ValueError('请选择 PKL 关节顺序')
            options['joint_order'] = order
        else:
            fps = x.get('fps')
            if fps is not None and (type(fps) not in (int,float) or not math.isfinite(fps) or not 0 < fps <= 1000):
                raise ValueError('无效源帧率')
            if row['kind'] == 'smpl' and fps is None: raise ValueError('PT 必须明确源帧率')
            options['fps'] = fps
            group = x.get('group','auto')
            if group not in ('auto','body_params_global','body_params_incam'): raise ValueError('无效 PT 字段组')
            options['group'] = group
        publication=None
        if 'output_name' in x:
            from .motion_publication import publication_spec
            publication=publication_spec(self,row,x['output_name'])
        return dict(action='convert', kind=row['kind'], input=str(path), root=str(self.root),
                    source_sha256=row['source_sha256'], options=options, retirement=retirement,
                    publication=publication)

    def convert(self, spec, cancelled):
        interpreter = ROOT/'.venv_import/bin/python' if spec.get('kind') == 'smpl' else Path(sys.executable)
        if not interpreter.is_file():
            raise ValueError('PT 独立导入环境尚未安装')
        prefix = '.import-'+secrets.token_hex(12)+'-'
        spec = dict(spec, staging_prefix=prefix)
        try:
            result, area = run_worker(spec, cancelled=cancelled, timeout=300, interpreter=interpreter)
            area.cleanup()
            if spec.get('publication'):
                from .motion_publication import publish_named
                return publish_named(self,spec,result,cancelled)
            retirement=spec.get('retirement')
            if retirement:
                if cancelled():raise ValueError('任务已取消，旧缓存未移除')
                replacement=inside(self.root,result['motion'])
                clip=validated_cache(self.root,result['motion'])
                meta=cache_metadata(replacement,clip)
                if meta.get('source_sha256')!=spec['source_sha256']:
                    raise ValueError('新缓存来源不匹配，旧缓存未移除')
                old=inside(self.root,retirement['motion'])
                if old==replacement:raise ValueError('重转结果不能覆盖旧缓存')
                if cache_identity(old)!=retirement['identity']:
                    raise ValueError('旧缓存内容已变化，请刷新后重试')
                archive=self.root.parent/'motion-archive'/(time.strftime('%Y%m%dT%H%M%S',time.gmtime())+'-'+secrets.token_hex(4))
                archive.mkdir(parents=True,mode=0o700)
                old.rename(archive/old.name)
                result=dict(result,retired_motion=retirement['motion'],archive=str(archive/old.name))
            return result
        finally:
            # SIGKILL cannot run the worker's TemporaryDirectory finalizer.
            # Remove only this task's unpredictable staging prefix, after wait().
            for path in self.root.glob(prefix+'*'):
                if path.is_symlink() or path.is_file():path.unlink(missing_ok=True)
                else:shutil.rmtree(path, ignore_errors=True)
