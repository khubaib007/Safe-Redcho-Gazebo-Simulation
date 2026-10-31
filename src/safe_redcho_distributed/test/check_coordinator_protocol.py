"""Exercise the coordinator with lightweight ROS stubs; no native runtime claim."""
import sys
import types
from pathlib import Path
from concurrent.futures import Future
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
class Message:
    pass
class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,m):self.messages.append(m)
    def get_subscription_count(self):return 6
class Logger:
    def info(self,s):pass
class Node:
    def __init__(self,*args):pass
    def create_publisher(self,*args):return Publisher()
    def create_subscription(self,*args):return None
    def create_timer(self,*args):return None
    def create_client(self,*args):return Client()
    def get_logger(self):return Logger()
class Client:
    def __init__(self):self.requests=[]
    def service_is_ready(self):return True
    def call_async(self,r):self.requests.append(r);self.future=Future();return self.future
def module(name,**fields):
    m=types.ModuleType(name);m.__dict__.update(fields);sys.modules[name]=m
module('rclpy');module('rclpy.node',Node=Node)
module('rclpy.qos',QoSProfile=lambda **kw:None,ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1))
module('safe_redcho_distributed_interfaces');module('safe_redcho_distributed_interfaces.msg',
       **{name:Message for name in ['ControlFrame','ControlPrepared','SafetyBounds','ControlResult','RingAck']})
module('safe_redcho_interfaces');module('safe_redcho_interfaces.srv',StepSwarm=types.SimpleNamespace(Request=Message))
import importlib.util
protocol=Path(sys.argv[1]) if len(sys.argv)>1 else None
if protocol is not None:
    spec=importlib.util.spec_from_file_location("safe_redcho_swarm.nodes.lockstep_protocol",protocol)
    loaded=importlib.util.module_from_spec(spec);spec.loader.exec_module(loaded)
    sys.modules[spec.name]=loaded
from safe_redcho_distributed.gazebo_mission import GazeboMission
from safe_redcho_distributed.local_controller import LocalController
from safe_redcho_distributed.ring_protocol import neighbors
from safe_redcho_distributed.control_replay import DIAGNOSTIC_KEYS
import json

def reply(sequence,ns,state,forces,limits,steps=10,wrenches=10):
    def xyz(v):return types.SimpleNamespace(**dict(zip('xyz',v)))
    stamp=types.SimpleNamespace(sec=ns//10**9,nanosec=ns%10**9)
    p,v,R,w=state;states=[]
    for i in range(6):
        q=types.SimpleNamespace(**dict(zip('xyzw',Rotation.from_matrix(R[:,:,i]).as_quat())))
        states.append(types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp,frame_id='world'),
            child_frame_id=f'cf{i+1}/base_link',pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=xyz(p[:,i]),orientation=q)),
            twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=xyz(R[:,:,i].T@v[:,i]),angular=xyz(w[:,i])))))
    return types.SimpleNamespace(success=True,status='OK',sequence=sequence,stamp=stamp,states=states,
        physics_steps=steps,wrench_steps=wrenches,applied_forces=forces.ravel().tolist(),applied_limits=limits.tolist())

def run():
    n=GazeboMission(0.);n.ready=set(range(1,7));n.pids={i:100+i for i in n.ready}
    n.advance();assert n.phase=='bootstrap' and len(n.client.requests)==1
    state=tuple(x.copy() for x in n.state)
    n.client.future.set_result(reply(0,1_000_000,state,np.zeros((6,4)),np.full(6,.16),1,0))
    n.advance();assert n.current==1 and n.phase=='control'
    frame=n.pub.messages[-1]
    assert np.array_equal(np.asarray(frame.positions).reshape(3,6),state[0])
    assert np.allclose(np.asarray(frame.velocities).reshape(3,6),state[1],atol=1e-15)
    agents={i:LocalController(i) for i in range(1,7)}
    prepared={i:a.prepare(1,0.,n.state,np.zeros((3,6))) for i,a in agents.items()}
    for i in range(1,7):
        m=Message();m.agent_id=i;m.cycle=1;m.acceleration_bound=prepared[i][1];n.prepare(i,m)
    for i in range(1,7):
        a=agents[i];d=a.finish({peer:prepared[peer][0] for peer in neighbors(i)},np.array([prepared[j][1] for j in range(1,7)]))
        m=Message();m.agent_id=i;m.cycle=1;m.valid=True;m.detail=json.dumps({'pid':100+i})
        m.safe_acceleration=d['u_safe_hist'];m.motor_forces=d['u_motors_hist'];m.motor_limit=d['F_max_hist']
        m.diagnostics_json=json.dumps({key:np.asarray(d[key]).tolist() for key in DIAGNOSTIC_KEYS})
        if i==6:n.advance();assert len(n.client.requests)==1 # missing last command: no step
        n.receive(i,m)
    assert n.error is None,n.error
    n.advance();assert n.phase=='step' and len(n.client.requests)==2
    forces,limits=n.batch
    assert np.array_equal(np.asarray(n.client.requests[-1].forces).reshape(6,4),forces)
    n.advance();assert n.completed==0 and len(n.pub.messages)==1 # pending response: no next frame
    n.client.future.set_result(reply(1,11_000_000,n.state,forces,limits))
    n.advance();assert n.finished and n.completed==1 and n.accepted==1 and n.error is None
    print('PASS: bootstrap, body/world conversion, six-command barrier, motor ordering, pending-step barrier and terminal-cycle accounting')

if __name__=='__main__':run()
