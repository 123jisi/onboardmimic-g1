"""Offline, trusted-file import. Never opens a control socket.

GMR conventions and finite differences ported from genmo-teleop/sonic_extra/
convert_gmr_to_sonic.py. SMPL semantics follow publish_smpl_params.py and the
vendored SONIC FK/rest data (pico/pose_sources.json and licenses/SONIC_LICENSE).
"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import os

import numpy as np

from .csv_player import REFERENCE_TO_HARDWARE

VERSION = 'onboard-motion-1'
MAX_FRAMES = 30000


def conversion_fingerprint():
    root = Path(__file__).parent
    return hashlib.sha256(b"".join((root/name).read_bytes() for name in
        ("motion_import.py", "pico/human_rest.json", "pico/wrist_mapping.py", "pico/pose_mapping.py", "kimodo_import.py"))).hexdigest()


def finite_diff(a, fps):
    v = np.empty_like(a, dtype=np.float32)
    dt = 1./fps
    v[0], v[-1] = (a[1]-a[0])/dt, (a[-1]-a[-2])/dt
    v[1:-1] = (a[2:]-a[:-2])/(2*dt)
    return v


def checked(a, shape, name):
    a = np.asarray(a, dtype=np.float32)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError('Invalid shape or nonfinite values: '+name)
    return a


def time_axes(frames, fps):
    if not 2 <= frames <= MAX_FRAMES or not np.isfinite(fps) or not 0 < fps <= 1000:
        raise ValueError('Invalid frames/fps')
    duration = (frames-1)/fps
    count = int(round(duration*50))+1
    if not 2 <= count <= MAX_FRAMES:
        raise ValueError('Resampled frame count exceeds player limit')
    # Preserve upstream endpoint-inclusive interpolation. The 50 Hz player
    # holds the last sample for one tick; timing quantization is <= 30 ms.
    return np.arange(frames)/fps, np.linspace(0, duration, count)


def linear(a, ti, to):
    flat = a.reshape(len(a), -1)
    return np.stack([np.interp(to, ti, c) for c in flat.T], axis=-1).reshape((len(to),)+a.shape[1:]).astype(np.float32)


def rotations(a, ti, to):
    from scipy.spatial.transform import Rotation as R, Slerp
    # a is xyzw; also used independently for each SMPL local joint.
    q = Slerp(ti, R.from_quat(a))(to).as_quat().astype(np.float32)
    for i in range(1, len(q)):
        if np.dot(q[i-1], q[i]) < 0:
            q[i] *= -1
    return q


def source_info(path, kind, fps, frames, **options):
    return dict(version=VERSION, source_format=kind, source_name=path.name,
                source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                conversion_sha256=conversion_fingerprint(),
                source_fps=fps, source_frames=frames, fps=50.,
                source_duration_seconds=(frames-1)/fps, options=options)


def import_gmr(path, output, joint_order):
    """Convert G1 GMR qpos[7:] to SONIC's interleaved reference order."""
    from scipy.spatial.transform import Rotation as R
    import joblib  # import dependency only; pickle/joblib must be trusted
    path = Path(path)
    if path.stat().st_size > 128*1024*1024: raise ValueError('Input exceeds 128 MiB')
    data = joblib.load(path)
    if not isinstance(data, dict):
        raise ValueError('GMR requires a dict')
    fps = float(data['fps'])
    q = np.asarray(data['dof_pos'], dtype=np.float32)
    if q.ndim != 2:
        raise ValueError('dof_pos must be (T,29)')
    n = len(q)
    ti, to = time_axes(n, fps)
    q = checked(q, (n,29), 'dof_pos')
    p = checked(data['root_pos'], (n,3), 'root_pos')
    quat = checked(data['root_rot'], (n,4), 'root_rot xyzw')
    if np.any(np.abs(np.linalg.norm(quat, axis=1)-1) > .02):
        raise ValueError('root_rot must contain unit xyzw quaternions')
    if joint_order not in ('mujoco', 'isaaclab'):
        raise ValueError('Explicit input joint order required')
    # GMR's MuJoCo qpos[7:] is grouped in hardware order. SONIC's
    # MotionSequence/encoder expects the interleaved policy order. The native
    # mapping names are historical; use the actual gather, not those labels.
    # An IsaacLab-ordered input is already in that interleaved reference order.
    if joint_order == 'mujoco':
        q = q[:, REFERENCE_TO_HARDWARE]
    if abs(fps-50) > 1e-6:
        q, p, quat = linear(q,ti,to), linear(p,ti,to), rotations(quat,ti,to)
    quat = quat[:, [3,0,1,2]]
    ang = np.zeros((len(q),3), dtype=np.float32)
    r = R.from_quat(quat[:,[1,2,3,0]])
    ang[1:] = (r[1:]*r[:-1].inv()).as_rotvec()*50
    ang[0] = ang[1]
    arrays = dict(joint_pos=q, joint_vel=finite_diff(q,50), body_pos=p,
                  body_quat=quat, body_lin_vel=finite_diff(p,50), body_ang_vel=ang)
    meta = source_info(path,'gmr_pkl',fps,n,input_joint_order=joint_order,root_rotation='xyzw',
                       joint_mapping='gmr_to_sonic_reference_v2',
                       source_column_gather=list(REFERENCE_TO_HARDWARE) if joint_order == 'mujoco' else list(range(29)))
    meta.update(frames=len(q), joint_order='sonic_reference', body_semantics='root-only world pose/spatial angular velocity')
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Publish a complete directory, never overwrite an already loaded action.
    with tempfile.TemporaryDirectory(dir=output.parent) as temp:
        stage = Path(temp)/'motion'
        stage.mkdir()
        for name,a in arrays.items():
            np.savetxt(stage/(name+'.csv'), a, delimiter=',', fmt='%.6f',
                       header=','.join(name+'_'+str(i) for i in range(a.shape[1])), comments='')
        (stage/'metadata.txt').write_text('Body part indexes:\n[0]\nTotal timesteps: '+str(len(q))+'\n')
        (stage/'import.json').write_text(json.dumps(meta, indent=2))
        if output.exists():
            raise ValueError('Output already exists')
        stage.rename(output)
    return meta


def smpl_arrays(data, wrist_mapping='genmo'):
    from scipy.spatial.transform import Rotation as R
    from .pico.pose_mapping import J, FK_PARENTS, BASE, UP
    from .pico.wrist_mapping import compute_wrist_joints
    pose = np.asarray(data.get('smpl_pose', data.get('body_pose')), dtype=np.float32)
    if pose.ndim == 2 and pose.shape[1] >= 63:
        pose = pose[:,:63].reshape(-1,21,3)
    n = len(pose)
    if not 2 <= n <= MAX_FRAMES: raise ValueError('Invalid SMPL frame count')
    pose = checked(pose,(n,21,3),'body_pose')
    orient = checked(np.asarray(data.get('global_orient', np.zeros((n,3)))).reshape(-1,3),(n,3),'global_orient')
    root = UP*R.from_rotvec(orient)
    body = root*BASE.inv()
    quat = body.as_quat()[:,[3,0,1,2]].astype(np.float32)
    for i in range(1,n):
        if np.dot(quat[i-1],quat[i]) < 0: quat[i] *= -1
    joints = data.get('smpl_joints', data.get('smpl_joints_local'))
    if joints is None:
        local = np.tile(np.eye(3),(n,55,1,1))
        local[:,0] = root.as_matrix()
        local[:,1:22] = R.from_rotvec(pose.reshape(-1,3)).as_matrix().reshape(n,21,3,3)
        wm, wp = np.empty_like(local), np.empty((n,55,3))
        for i,parent in enumerate(FK_PARENTS):
            if parent < 0: wm[:,i],wp[:,i] = local[:,i],J[i]
            else:
                wm[:,i] = wm[:,parent]@local[:,i]
                wp[:,i] = wp[:,parent]+np.einsum('nij,j->ni',wm[:,parent],J[i]-J[parent])
        joints = np.einsum('nij,nkj->nki',body.inv().as_matrix(),wp[:,list(range(22))+[39,54]])
    joints = checked(joints,(n,24,3),'smpl_joints')
    if wrist_mapping not in ('genmo', 'kimodo'):
        raise ValueError('Unknown wrist mapping')
    q = np.zeros((n,29),dtype=np.float32)
    for i,p in enumerate(pose):
        left,right = compute_wrist_joints(p.reshape(63))
        if wrist_mapping == 'kimodo':
            # Actual assignments in publish_smpl_kimodo.py; its comment lists
            # a different order. Kimodo also retains the negative left pitch.
            q[i,[23,25,27]] = [left[0], -left[1], left[2]]
            q[i,[24,26,28]] = right
        else:
            q[i,23:26],q[i,26:29] = left,right
    return dict(smpl_pose=pose,smpl_joints=joints,body_quat_w=quat,joint_pos=q,
                joint_vel=np.zeros_like(q))


def import_pt(path, output, fps, group='auto'):
    import torch  # isolated CPU import environment; never imported by player
    path,output = Path(path),Path(output)
    if path.stat().st_size > 128*1024*1024: raise ValueError('Input exceeds 128 MiB')
    # Safe tensor/primitive loader. Never fall back to arbitrary pickle code.
    raw = torch.load(path, map_location='cpu', weights_only=True)
    if not isinstance(raw,dict): raise ValueError('PT requires a dict')
    selected = group
    if group == 'auto':
        selected = next((k for k in ('body_params_global','body_params_incam') if isinstance(raw.get(k),dict)), None) if not any(k in raw for k in ('body_pose','smpl_pose')) else None
    data = raw[selected] if selected else raw
    if not isinstance(data,dict): raise ValueError('Unsupported SMPL group')
    if not np.isfinite(fps) or not 0 < fps <= 1000: raise ValueError('Invalid source FPS')
    data = {k:(v.detach().cpu().numpy() if isinstance(v,torch.Tensor) else v) for k,v in data.items()}
    arrays = smpl_arrays(data)
    n = len(arrays['smpl_pose'])
    time_axes(n,float(fps))
    meta = source_info(path,'smpl_pt',float(fps),n,group=group,selected_group=selected,
                       skeleton_sha256=hashlib.sha256(Path(__file__).with_name('pico').joinpath('human_rest.json').read_bytes()).hexdigest())
    return write_smpl_cache(output, arrays, meta)


def write_smpl_cache(output, arrays, meta, wrist_mapping='genmo'):
    from scipy.spatial.transform import Rotation as R
    output = Path(output)
    fps = meta['source_fps']
    ti,to = time_axes(len(arrays['smpl_pose']),fps)
    # Keep original samples for exact source comparison and interpolate at import.
    if abs(fps-50) > 1e-6:
        pose = np.stack([R.from_quat(rotations(R.from_rotvec(arrays['smpl_pose'][:,j]).as_quat(),ti,to)).as_rotvec() for j in range(21)],axis=1).astype(np.float32)
        interp = smpl_arrays(dict(smpl_pose=pose, smpl_joints=linear(arrays['smpl_joints'],ti,to)), wrist_mapping=wrist_mapping)
        interp['body_quat_w'] = rotations(arrays['body_quat_w'][:,[1,2,3,0]],ti,to)[:,[3,0,1,2]]
    else: interp = arrays
    meta.update(frames=len(interp['smpl_pose']), representation='smpl', shapes={k:list(v.shape) for k,v in interp.items()})
    output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists(): raise ValueError('Output already exists')
    with tempfile.NamedTemporaryFile(dir=output.parent, suffix='.npz', delete=False) as f:
        temp = Path(f.name)
        try:
            np.savez_compressed(f, metadata=np.array(json.dumps(meta)), **interp,
                                **{'source_'+k:v for k,v in arrays.items()})
            f.close()
            os.link(temp,output)  # exclusive, complete publication
        finally: temp.unlink(missing_ok=True)
    return meta


def main():
    p = argparse.ArgumentParser(description='Trusted local PKL/PT/AMASS import; no controller or network')
    sub = p.add_subparsers(dest='kind',required=True)
    g = sub.add_parser('gmr'); g.add_argument('input',type=Path); g.add_argument('output',type=Path)
    g.add_argument('--input-joint-order',choices=['mujoco','isaaclab'],required=True,
                   help='mujoco: GMR G1 qpos[7:] / hardware order, reordered to SONIC; '
                        'isaaclab: already in SONIC interleaved reference order')
    s = sub.add_parser('smpl'); s.add_argument('input',type=Path); s.add_argument('output',type=Path)
    s.add_argument('--fps',type=float,required=True,help='Actual source FPS, not publish rate')
    s.add_argument('--group',choices=['auto','body_params_global','body_params_incam'],default='auto')
    k = sub.add_parser('kimodo', help='Kimodo AMASS NPZ -> local SMPL v3 cache; no Torch')
    k.add_argument('input',type=Path); k.add_argument('output',type=Path)
    k.add_argument('--fps',type=float,help='Override actual source FPS; otherwise read NPZ metadata')
    a = p.parse_args()
    if a.kind == 'gmr':
        meta = import_gmr(a.input,a.output,a.input_joint_order)
    elif a.kind == 'smpl':
        meta = import_pt(a.input,a.output,a.fps,a.group)
    else:
        from .kimodo_import import import_kimodo
        meta = import_kimodo(a.input,a.output,a.fps)
    print(json.dumps(meta,indent=2))


if __name__ == '__main__': main()
