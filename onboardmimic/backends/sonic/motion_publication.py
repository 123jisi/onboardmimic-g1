"""Named conversion publication. No controller, DDS or hand output dependencies."""
import os
from pathlib import Path
import re
import secrets
import shutil


def output_name(value, kind):
    if not isinstance(value,str):raise ValueError('请填写动作名称')
    value=value.strip()
    if kind!='gmr' and value.endswith('.npz'):value=value[:-4]
    if not re.fullmatch(r'[\w][\w .-]{0,79}',value) or value.endswith('.') or len(value.encode())>240:
        raise ValueError('动作名称须为1至80个中文、字母、数字、空格、下划线、短横线或点，且不能以点开头或结尾')
    return value+('' if kind=='gmr' else '.npz')


def publication_spec(library,row,value):
    from .motion_library import inside,cache_identity
    name=output_name(value,row['kind'])
    fmt={'gmr':'gmr_pkl','smpl':'smpl_pt','kimodo':'kimodo_amass_npz'}[row['kind']]
    previous=[]
    for entry in library.entries:
        meta=entry.get('metadata',{})
        if entry['kind']=='cache' and meta.get('source_sha256')==row['source_sha256'] and meta.get('source_format')==fmt:
            previous.append(dict(motion=entry['motion'],identity=cache_identity(inside(library.root,entry['motion']))))
    target=library.root/name
    if os.path.lexists(str(target)) and name not in {p['motion'] for p in previous}:
        raise ValueError('动作名称已被其他文件占用，请换个名称')
    return dict(name=name,source_format=fmt,previous=previous)


def publish_named(library,spec,result,cancelled):
    from .motion_library import inside,cache_identity,validated_cache,cache_metadata,digest,scan_catalog
    root=library.root;publication=spec['publication'];name=publication['name']
    stage=inside(root,result['motion'])
    if not stage.name.startswith(spec['staging_prefix']):raise ValueError('转换临时结果不匹配')
    clip=validated_cache(root,stage.name);meta=cache_metadata(stage,clip)
    if meta.get('source_sha256')!=spec['source_sha256'] or meta.get('source_format')!=publication['source_format']:
        raise ValueError('新缓存来源不匹配，旧结果未删除')
    if digest(spec['input'])!=spec['source_sha256']:raise ValueError('转换期间源文件已变化')
    previous=publication['previous'];names={p['motion'] for p in previous}
    # Refuse a stale catalog or any independent writer's new same-source cache.
    current={e['motion'] for e in scan_catalog(root,library.source_root) if e['kind']=='cache' and
             e.get('metadata',{}).get('source_sha256')==spec['source_sha256'] and
             e.get('metadata',{}).get('source_format')==publication['source_format']}
    if current!=names:raise ValueError('同源结果已变化，请刷新动作库后重试')
    for old in previous:
        path=inside(root,old['motion'])
        if cache_identity(path)!=old['identity']:raise ValueError('旧缓存内容已变化，请刷新后重试')
    target=root/name
    if os.path.lexists(str(target)) and name not in names:raise ValueError('动作名称已被其他文件占用，请换个名称')
    if cancelled():raise ValueError('任务已取消，旧结果未删除')
    rollback=root/('.replacement-'+secrets.token_hex(12));rollback.mkdir(mode=0o700)
    moved=[]
    try:
        for old in previous:
            original=root/old['motion'];original.rename(rollback/original.name);moved.append(original.name)
        if os.path.lexists(str(target)):raise ValueError('动作名称已被其他文件占用，请换个名称')
        if stage.is_dir():stage.rename(target)
        else:os.link(str(stage),str(target));stage.unlink()
    except BaseException:
        # No old data is deleted until a complete new cache is visible.
        for old_name in reversed(moved):
            saved=rollback/old_name;original=root/old_name
            if os.path.lexists(str(original)):
                raise RuntimeError('发布冲突，旧结果保留在 '+str(rollback))
            saved.rename(original)
        rollback.rmdir()
        raise
    answer=dict(result,motion=name,reused=False,metadata=meta,replaced_motions=sorted(names))
    # Commit succeeded. Retired caches are actually removed, not kept as versions.
    try:shutil.rmtree(str(rollback))
    except OSError as exc:
        answer.update(cleanup_pending=str(rollback),warning='新结果已发布，旧文件清理未完成：'+str(exc))
    return answer
