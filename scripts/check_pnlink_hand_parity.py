"""Compare hand math with the actual host source; imports no live host modules."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
from scipy.spatial.transform import Rotation
from onboardmimic.backends.sonic.pnlink.hand_mapping import Mapping


def compare(reference_root, fixture=None):
    source=Path(reference_root)/'host_setup/g1_hands/mocap.py'
    tree=ast.parse(source.read_text())
    names={'CHANNELS','FINGERS','GROUPS','FLEX_GROUPS','CHANNEL_FINGERS','MIN_SPAN','CAPS'}
    nodes=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in ('pose_array','relative_pose','Mapping')
           or isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in names for t in n.targets)]
    env=dict(np=np,Rotation=Rotation,math=__import__('math'))
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(source),'exec'),env)
    rng=np.random.default_rng(731)
    opened=rng.normal(0,.15,(15,3));closed=(Rotation.from_rotvec(opened)*Rotation.from_rotvec(rng.normal(.3,.08,(15,3)))).as_rotvec()
    calibrations=[dict(version=v,hand='right',open=opened.tolist(),closed=closed.tolist()) for v in (1,2)]
    poses=[(Rotation.from_rotvec(opened)*Rotation.from_rotvec(rng.normal(.2,.3,(15,3)))).as_rotvec().tolist() for _ in range(24)]
    cases=[];error=0.
    for calibration in calibrations:
        for mode in ('fingers','grasp'):
            for closure in (.05,.25,.7,1.):
                actual=Mapping(calibration,mode,closure);reference=env['Mapping'](calibration,mode,closure)
                expected=[reference.target(p).tolist() for p in poses]
                error=max(error,max(float(np.max(np.abs(actual.target(p)-q))) for p,q in zip(poses,expected)))
                cases.append(dict(version=calibration['version'],mode=mode,closure=closure,targets=expected))
    result=dict(passed=error<1e-12,poses=len(poses),comparisons=len(poses)*len(cases),max_target_error=error,
                reference_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),sdk_loaded=False,motor_output=False)
    if fixture:Path(fixture).write_text(json.dumps(dict(validation=result,calibrations=calibrations,poses=poses,cases=cases),indent=2)+'\n')
    if not result['passed']:raise AssertionError(result)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference-root',required=True,type=Path);p.add_argument('--fixture',type=Path)
    a=p.parse_args();print(json.dumps(compare(a.reference_root,a.fixture),indent=2))
