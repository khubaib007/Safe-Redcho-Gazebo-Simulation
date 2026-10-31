"""One complete local controller; commands published to a replay or Gazebo coordinator."""
import json
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from safe_redcho_distributed_interfaces.msg import ControlFrame,ControlPrepared,SafetyBounds,ControlResult,ObserverNeighbor,RingAck,PoseRedchoNeighbor
from .local_controller import LocalController
from . import scenario


def json_ready(values):
    return {k:np.asarray(v).tolist() for k,v in values.items()}


class ControlAgent(Node):
    def __init__(self):
        super().__init__('safe_redcho_control_agent')
        self.declare_parameter('agent_id',1)
        self.agent_id=int(self.get_parameter('agent_id').value);self.model=LocalController(self.agent_id)
        self.pending={};self.formation_pending={};self.bounds={};self.failed=False
        qos=QoSProfile(depth=16,reliability=ReliabilityPolicy.RELIABLE)
        self.pub=self.create_publisher(ObserverNeighbor,f'/cf{self.agent_id}/control_redcho_neighbor',qos)
        self.formation_pub=self.create_publisher(PoseRedchoNeighbor,f'/cf{self.agent_id}/control_pose_redcho_neighbor',qos)
        self.prepared=self.create_publisher(ControlPrepared,f'/cf{self.agent_id}/control_prepared',qos)
        self.results=self.create_publisher(ControlResult,f'/cf{self.agent_id}/control_result',qos)
        self.ready_pub=self.create_publisher(RingAck,'/safe_redcho_distributed/control_ready',qos)
        self.input=self.create_subscription(ControlFrame,'/safe_redcho_distributed/control_frame',self.begin,qos)
        self.bound_sub=self.create_subscription(SafetyBounds,'/safe_redcho_distributed/safety_bounds',self.safety,qos)
        self.subs=[self.create_subscription(ObserverNeighbor,f'/cf{peer}/control_redcho_neighbor',
                   lambda msg,peer=peer:self.receive(peer,msg),qos) for peer in self.model.neighbors]
        self.subs += [self.create_subscription(PoseRedchoNeighbor,f'/cf{peer}/control_pose_redcho_neighbor',
                    lambda msg,peer=peer:self.receive_formation(peer,msg),qos) for peer in self.model.neighbors]
        self.timer=self.create_timer(.2,self.ready)
        self.get_logger().info(f'{scenario.IMPLEMENTATION_VERSION} | agent={self.agent_id}, pid={os.getpid()}, velocity REDCHO + position/height REDCHO + local HOCBF-QP, ring={self.model.neighbors}')

    def ready(self):
        if (self.model.observer.cycle==0 and not self.failed and self.pub.get_subscription_count()==2
                and self.formation_pub.get_subscription_count()==2
                and self.results.get_subscription_count()>=1 and self.prepared.get_subscription_count()==1):
            msg=RingAck();msg.cycle=0;msg.agent_id=self.agent_id;msg.neighbor_ids=list(self.model.neighbors)
            msg.valid=True;msg.detail=json.dumps({'pid':os.getpid()});self.ready_pub.publish(msg)

    def begin(self,msg):
        if self.failed:return
        try:
            state=(np.asarray(msg.positions).reshape(3,6),np.asarray(msg.velocities).reshape(3,6),
                   np.asarray(msg.rotations).reshape(3,3,6),np.asarray(msg.angular_velocities).reshape(3,6))
            outgoing,bound=self.model.prepare(int(msg.cycle),float(msg.mission_time),state,np.asarray(msg.previous_accelerations).reshape(3,6))
            self.pending={k:v for k,v in self.pending.items() if k>=msg.cycle}
            self.formation_pending={k:v for k,v in self.formation_pending.items() if k>=msg.cycle}
            self.bounds={k:v for k,v in self.bounds.items() if k>=msg.cycle}
            n=ObserverNeighbor();n.cycle=msg.cycle;n.agent_id=self.agent_id
            n.svf_velocity_estimate=outgoing[0].tolist();n.levant_velocity_estimate=outgoing[1].tolist();self.pub.publish(n)
            estimate=self.model.formation_packet()
            f=PoseRedchoNeighbor();f.cycle=msg.cycle;f.agent_id=self.agent_id;f.mission_time=msg.mission_time
            f.displacement_estimate=estimate.tolist();self.formation_pub.publish(f)
            p=ControlPrepared();p.cycle=msg.cycle;p.agent_id=self.agent_id;p.acceleration_bound=float(bound);self.prepared.publish(p)
            self.finish()
        except Exception as exc:self.fail(str(exc))

    def receive(self,peer,msg):
        if self.failed:return
        try:
            if msg.agent_id!=peer:raise ValueError('Neighbor/topic identity mismatch')
            cycle=int(msg.cycle);current=self.model.observer.cycle
            if cycle<current:return
            if cycle<1 or cycle>current+1:raise ValueError('Neighbor cycle mismatch')
            peers=self.pending.setdefault(cycle,{})
            if peer in peers:raise ValueError('Duplicate neighbor message')
            peers[peer]=(np.asarray(msg.svf_velocity_estimate),np.asarray(msg.levant_velocity_estimate));self.finish()
        except Exception as exc:self.fail(str(exc))

    def safety(self,msg):
        if self.failed:return
        try:
            if msg.cycle!=self.model.observer.cycle:raise ValueError('Safety bound cycle mismatch')
            if msg.cycle in self.bounds:raise ValueError('Duplicate safety bounds')
            self.bounds[msg.cycle]=np.asarray(msg.acceleration_bounds);self.finish()
        except Exception as exc:self.fail(str(exc))

    def receive_formation(self,peer,msg):
        if self.failed:return
        try:
            if msg.agent_id!=peer:raise ValueError('Formation neighbor/topic identity mismatch')
            cycle=int(msg.cycle);current=self.model.observer.cycle
            if cycle<current:return
            if cycle<1 or cycle>current+1:raise ValueError('Formation neighbor cycle mismatch')
            if not np.isfinite(msg.mission_time) or abs(msg.mission_time-(cycle-1)*self.model.dt)>1e-9:
                raise ValueError('Formation neighbor timestamp mismatch')
            estimate=np.asarray(msg.displacement_estimate)
            if estimate.shape!=(3,) or not np.isfinite(estimate).all():
                raise ValueError('Invalid formation neighbor state')
            peers=self.formation_pending.setdefault(cycle,{})
            if peer in peers:raise ValueError('Duplicate formation neighbor message')
            peers[peer]=estimate.copy();self.finish()
        except Exception as exc:self.fail(str(exc))

    def finish(self):
        if self.model.finished:return
        cycle=self.model.observer.cycle;peers=self.pending.get(cycle,{})
        formation_peers=self.formation_pending.get(cycle,{})
        if (cycle not in self.bounds or set(peers)!=set(self.model.neighbors)
                or set(formation_peers)!=set(self.model.neighbors)):return
        d=self.model.finish(peers,self.bounds[cycle],formation_peers)
        msg=ControlResult();msg.cycle=cycle;msg.agent_id=self.agent_id;msg.valid=True
        msg.detail=json.dumps({'pid':os.getpid()});msg.safe_acceleration=d['u_safe_hist'].tolist()
        msg.motor_forces=d['u_motors_hist'].tolist();msg.motor_limit=float(d['F_max_hist'])
        msg.diagnostics_json=json.dumps(json_ready(d),allow_nan=False);self.results.publish(msg)

    def fail(self,error):
        self.failed=True;self.get_logger().error(error)
        msg=ControlResult();msg.cycle=self.model.observer.cycle;msg.agent_id=self.agent_id;msg.valid=False
        msg.detail=json.dumps({'pid':os.getpid(),'error':error});self.results.publish(msg)


def main(args=None):
    rclpy.init(args=args);node=ControlAgent()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
