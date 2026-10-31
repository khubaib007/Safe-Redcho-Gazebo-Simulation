"""Standalone protocol checks: python3 test/check_lockstep_protocol.py."""
from pathlib import Path
from types import SimpleNamespace as S
import importlib.util
import copy
import numpy as np

path=Path(__file__).resolve().parents[1]/'safe_redcho_swarm/nodes/lockstep_protocol.py'
spec=importlib.util.spec_from_file_location('protocol',path)
protocol=importlib.util.module_from_spec(spec)
spec.loader.exec_module(protocol)
stamp=S(sec=0,nanosec=10_000_000)
def vector(x=0.,y=0.,z=0.):return S(x=x,y=y,z=z)
states=[]
for i in range(6):
    states.append(S(
        header=S(stamp=copy.deepcopy(stamp),frame_id='world'),
        child_frame_id=f'cf{i+1}/base_link',
        pose=S(pose=S(position=vector(z=1),
                      orientation=S(x=0.,y=0.,z=2**-.5,w=2**-.5))),
        twist=S(twist=S(linear=vector(x=1.),angular=vector()))))
forces=np.full((6,4),.07332975)
limits=np.full(6,.16)
reply=S(success=True,status='ok',sequence=1,stamp=stamp,states=states,physics_steps=10,wrench_steps=10,
        applied_forces=forces.ravel().tolist(),applied_limits=limits.tolist())
ns,pos,vel,R,w=protocol.validate_reply(reply,1,0,forces,limits)
assert ns==10_000_000
np.testing.assert_allclose(vel,np.tile([0.,1.,0.],(6,1)),atol=1e-14)
bad=[]
r=copy.deepcopy(reply);r.sequence=2;bad.append(r)
r=copy.deepcopy(reply);r.stamp.nanosec+=1;bad.append(r)
r=copy.deepcopy(reply);r.states[5].header.stamp.nanosec+=1;bad.append(r)
r=copy.deepcopy(reply);r.states[0].child_frame_id='cf2/base_link';bad.append(r)
r=copy.deepcopy(reply);r.states[0].header.frame_id='map';bad.append(r)
r=copy.deepcopy(reply);r.applied_forces[0]+=.001;bad.append(r)
r=copy.deepcopy(reply);r.applied_forces[0]=float('nan');bad.append(r)
r=copy.deepcopy(reply);r.applied_limits[0]=.15;bad.append(r)
r=copy.deepcopy(reply);r.states[0].pose.pose.orientation.w=2.;bad.append(r)
r=copy.deepcopy(reply);r.states.pop();bad.append(r)
r=copy.deepcopy(reply);r.success=False;bad.append(r)
r=copy.deepcopy(reply);r.physics_steps=9;bad.append(r)
r=copy.deepcopy(reply);r.wrench_steps=9;bad.append(r)
for r in bad:
    try:protocol.validate_reply(r,1,0,forces,limits)
    except RuntimeError:pass
    else:raise AssertionError('Bad reply accepted')
# Saturation must match the requested cap exactly.
request=np.full((6,4),.3)
capped=copy.deepcopy(reply);capped.applied_forces=np.full(24,.16).tolist()
protocol.validate_reply(capped,1,0,request,limits)
print('PASS: coordinate conversion, capped motor batch, and 13 invalid-reply cases')
