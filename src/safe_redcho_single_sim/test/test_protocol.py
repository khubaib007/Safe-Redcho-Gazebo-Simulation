"""Actual callback methods with explicit transport doubles; no native ROS claim."""
import importlib
import json
import sys
import types
from concurrent.futures import Future
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from safe_redcho_single_sim.local_controller import LocalController,initial_plant_state
from safe_redcho_single_sim.protocol import identity


class Message:pass


class Publisher:
    def __init__(self,topic):self.topic=topic;self.messages=[]
    def publish(self,msg):self.messages.append(msg)
    def get_subscription_count(self):
        if self.topic.endswith('/position_neighbor'):return 2
        return 6 if self.topic.endswith(('/frame','/bounds')) else 1


class Client:
    def __init__(self):self.requests=[]
    def service_is_ready(self):return True
    def call_async(self,req):self.requests.append(req);self.future=Future();return self.future


class Node:
    def __init__(self,*args):pass
    def declare_parameter(self,*args):pass
    def get_parameter(self,*args):return types.SimpleNamespace(value=1)
    def create_publisher(self,cls,topic,qos):return Publisher(topic)
    def create_subscription(self,*args):return args
    def create_timer(self,*args):pass
    def create_client(self,*args):return Client()
    def get_logger(self):return types.SimpleNamespace(info=lambda x:None,error=lambda x:None)


@pytest.fixture
def runtime(monkeypatch):
    def module(name,**fields):
        m=types.ModuleType(name);m.__dict__.update(fields);monkeypatch.setitem(sys.modules,name,m)
    module('rclpy');module('rclpy.node',Node=Node)
    module('rclpy.qos',QoSProfile=lambda **kw:None,ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1))
    module('safe_redcho_distributed_interfaces');module('safe_redcho_distributed_interfaces.msg',
        **{n:Message for n in ['ControlFrame','ControlPrepared','SafetyBounds','ControlResult','RingAck','PoseRedchoNeighbor']})
    module('safe_redcho_interfaces');module('safe_redcho_interfaces.srv',StepSwarm=types.SimpleNamespace(Request=Message))
    for name in ['agent','coordinator','gazebo_mission']:
        monkeypatch.delitem(sys.modules,'safe_redcho_single_sim.'+name,raising=False)
    return importlib.import_module('safe_redcho_single_sim.agent'),importlib.import_module('safe_redcho_single_sim.gazebo_mission')


def packet(peer=2,cycle=1,t=0.):
    m=Message();m.agent_id=peer;m.cycle=cycle;m.mission_time=t;m.displacement_estimate=[0.,0.,0.];return m


def test_agent_early_peer_and_complete_barrier(runtime):
    module,_=runtime;n=module.ControlAgent();n.ready();assert len(n.ready_pub.messages)==1
    n.receive(2,packet());state=initial_plant_state();m=Message();m.cycle=1;m.mission_time=0.
    m.positions=state[0].ravel();m.velocities=state[1].ravel();m.rotations=state[2].ravel();m.angular_velocities=state[3].ravel();m.previous_accelerations=np.zeros(18)
    n.begin(m);assert len(n.pub.messages)==1 and not n.failed
    b=Message();b.cycle=1;b.acceleration_bounds=[n.model.bound]*6;n.safety(b)
    assert not n.results.messages
    n.receive(6,packet(6));assert len(n.results.messages)==1 and n.results.messages[0].valid
    d=json.loads(n.results.messages[0].diagnostics_json)
    assert 'pose_redcho_estimate_hist' in d and 'Y_hist' not in d


@pytest.mark.parametrize('case',['duplicate','future','time','identity','nan'])
def test_agent_bad_packet_latches_failure(runtime,case):
    module,_=runtime;n=module.ControlAgent();m=packet()
    if case=='duplicate':n.receive(2,m)
    elif case=='future':m.cycle=3;m.mission_time=.02
    elif case=='time':m.mission_time=.01
    elif case=='identity':m.agent_id=6
    else:m.displacement_estimate=[np.nan,0,0]
    n.receive(2,m);assert n.failed and not n.results.messages[-1].valid


def reply(sequence,ns,state,forces,limits,steps=10,wrenches=10):
    nsobj=lambda **kw:types.SimpleNamespace(**kw)
    xyz=lambda v:nsobj(**dict(zip('xyz',v)))
    stamp=nsobj(sec=ns//10**9,nanosec=ns%10**9);p,v,R,w=state;states=[]
    for i in range(6):
        q=nsobj(**dict(zip('xyzw',Rotation.from_matrix(R[:,:,i]).as_quat())))
        states.append(nsobj(header=nsobj(stamp=stamp,frame_id='world'),child_frame_id=f'cf{i+1}/base_link',
            pose=nsobj(pose=nsobj(position=xyz(p[:,i]),orientation=q)),
            twist=nsobj(twist=nsobj(linear=xyz(R[:,:,i].T@v[:,i]),angular=xyz(w[:,i])))))
    return nsobj(success=True,status='OK',sequence=sequence,stamp=stamp,states=states,
        physics_steps=steps,wrench_steps=wrenches,applied_forces=forces.ravel().tolist(),applied_limits=limits.tolist())


def prepared_cycle(runtime):
    _,module=runtime;n=module.GazeboMission(0.);n.ready=set(range(1,7));n.pids={i:100+i for i in n.ready}
    n.advance();assert n.phase=='bootstrap'
    state=initial_plant_state();n.client.future.set_result(reply(0,1_000_000,state,np.zeros((6,4)),np.full(6,.16),1,0));n.advance()
    assert n.phase=='control' and n.current==1
    agents={i:LocalController(i) for i in range(1,7)}
    p={i:a.prepare(1,0.,n.state,np.zeros((3,6))) for i,a in agents.items()}
    for i in range(1,7):
        m=Message();m.agent_id=i;m.cycle=1;m.acceleration_bound=p[i][1];n.prepare(i,m)
    for i,a in agents.items():
        d=a.finish({j:p[j][0] for j in a.neighbors},np.array([p[j][1] for j in range(1,7)]))
        m=Message();m.agent_id=i;m.cycle=1;m.valid=True;m.detail=json.dumps(identity(100+i))
        m.safe_acceleration=d['u_safe_hist'];m.motor_forces=d['u_motors_hist'];m.motor_limit=d['F_max_hist']
        m.diagnostics_json=json.dumps({k:np.asarray(v).tolist() for k,v in d.items()})
        if i==6:n.advance();assert len(n.client.requests)==1
        n.receive(i,m)
    assert n.error is None;n.advance();assert n.phase=='step'
    return n


def test_coordinator_waits_for_native_ack_and_counts_one_batch(runtime):
    n=prepared_cycle(runtime);n.advance();assert n.completed==0 and len(n.pub.messages)==1
    forces,limits=n.batch;n.client.future.set_result(reply(1,11_000_000,n.state,forces,limits));n.advance()
    assert n.completed==n.accepted==1 and n.finished and n.error is None


@pytest.mark.parametrize('case',['sequence','time','wrenches','force','frame'])
def test_coordinator_rejects_invalid_native_response(runtime,case):
    n=prepared_cycle(runtime);forces,limits=n.batch;r=reply(1,11_000_000,n.state,forces,limits)
    if case=='sequence':r.sequence=2
    elif case=='time':r.stamp.nanosec+=1
    elif case=='wrenches':r.wrench_steps=9
    elif case=='force':r.applied_forces[0]+=.001
    else:r.states[0].header.frame_id='bad'
    n.client.future.set_result(r);n.advance()
    assert n.error and n.accepted==0 and n.completed==0 and len(n.pub.messages)==1
