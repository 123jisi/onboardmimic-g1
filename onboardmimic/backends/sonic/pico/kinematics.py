"""G1 wrist FK and original XR three-point calibration, NumPy/SciPy only.

Origins/axes are extracted from the source URDF (hash in g1_kinematics.json).
q uses the measured hardware order: legs, waist, left arm, right arm.
No meshes, simulation, Pinocchio, Torch, display or actuator access.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

MODEL=json.loads(Path(__file__).with_name('g1_kinematics.json').read_text())
OFFSETS=[R.from_euler('xyz',a,degrees=True) for a in ((0,0,-90),(90,0,0),(-90,0,180),(0,0,-90))]
UPPER_INDICES=[12,13,14,15,22,16,23,17,24,18,25,19,26,20,27,21,28]

def key_frames(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(29,) or not np.isfinite(q).all():raise ValueError('需要有效的29关节实测反馈')
    values=dict(zip(MODEL['body_joint_names'],q))
    children={j['child'] for j in MODEL['joints']}
    parents={j['parent'] for j in MODEL['joints']}
    world={p:(np.zeros(3),R.identity()) for p in parents-children}
    pending=list(MODEL['joints'])
    while pending:
        remaining=[]
        for j in pending:
            if j['parent'] not in world:remaining.append(j);continue
            p,r=world[j['parent']];origin=R.from_euler('xyz',j['rpy'])
            a=np.asarray(j['axis']);a=a/np.linalg.norm(a)
            rot=R.from_rotvec(a*values.get(j['name'],0.)) if j['type'] in ('revolute','continuous') else R.identity()
            world[j['child']]=(p+r.apply(j['xyz']),r*origin*rot)
        if len(remaining)==len(pending):raise ValueError('G1运动学树不完整')
        pending=remaining
    result=[]
    for name,offset in (('left_wrist_yaw_link',[.18,-.025,0]),('right_wrist_yaw_link',[.18,.025,0]),('torso_link',[0,0,.35])):
        p,r=world[name];result.append(np.r_[p+r.apply(offset),r.as_quat()[[3,0,1,2]]])
    return np.asarray(result)

def raw_three_points(body):
    body=np.asarray(body,dtype=float)
    if body.shape!=(24,7) or not np.isfinite(body).all():raise ValueError('需要有效的24关节追踪')
    Q=np.array([[-1,0,0],[0,0,1],[0,1,0.]])
    ids=[0,22,23,12]
    positions=body[ids,:3]@Q.T
    rotations=[R.from_matrix(Q@R.from_quat(body[i,3:]).as_matrix()@Q.T)*offset for i,offset in zip(ids,OFFSETS)]
    root=rotations[0].inv()
    return np.array([np.r_[root.apply(p-positions[0]),(root*r).as_quat()[[3,0,1,2]]] for p,r in zip(positions[1:],rotations[1:])])

class ThreePointCalibration:
    def __init__(self):self.neck=None
    def capture(self,body,q):
        raw=raw_three_points(body);ref=key_frames(q)
        if self.neck is None:self.neck=R.from_quat(raw[2,3:][[1,2,3,0]]).inv()
        self.offsets=[];self.rotations=[]
        for i in (0,1):
            self.offsets.append(self.neck.apply(raw[i,:3])-ref[i,:3])
            corrected=self.neck*R.from_quat(raw[i,3:][[1,2,3,0]])
            self.rotations.append(R.from_quat(ref[i,3:][[1,2,3,0]])*corrected.inv())
    def apply(self,body):
        if self.neck is None:raise ValueError('三点映射尚未标定')
        raw=raw_three_points(body);out=raw.copy()
        for i in (0,1):
            out[i,:3]=self.neck.apply(raw[i,:3])-self.offsets[i]
            out[i,3:]=(self.rotations[i]*self.neck*R.from_quat(raw[i,3:][[1,2,3,0]])).as_quat()[[3,0,1,2]]
        neck=self.neck*R.from_quat(raw[2,3:][[1,2,3,0]])
        out[2,3:]=neck.as_quat()[[3,0,1,2]]
        out[2,:3]=np.array([0,0,.05])+.35*neck.apply([0,0,1])
        return out.astype(np.float32)
