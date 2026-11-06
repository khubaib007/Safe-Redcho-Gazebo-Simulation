"""Independent local controller; one same-cycle position REDCHO ring channel."""
import json
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from safe_redcho_distributed_interfaces.msg import ControlFrame,ControlPrepared,SafetyBounds,ControlResult,RingAck,PoseRedchoNeighbor
from .local_controller import LocalController
from .protocol import topic,identity
from . import scenario


class ControlAgent(Node):
    def __init__(self):
        super().__init__('single_position_control_agent')
        self.declare_parameter('agent_id',1)
        value=self.get_parameter('agent_id').value
        if type(value) is not int:raise ValueError('agent_id must be an integer')
        self.agent_id=value;self.model=LocalController(value)
        self.pending={};self.bounds={};self.failed=False
        qos=QoSProfile(depth=16,reliability=ReliabilityPolicy.RELIABLE)
        self.pub=self.create_publisher(PoseRedchoNeighbor,topic(f'cf{value}/position_neighbor'),qos)
        self.prepared=self.create_publisher(ControlPrepared,topic(f'cf{value}/prepared'),qos)
        self.results=self.create_publisher(ControlResult,topic(f'cf{value}/result'),qos)
        self.ready_pub=self.create_publisher(RingAck,topic('ready'),qos)
        self.create_subscription(ControlFrame,topic('frame'),self.begin,qos)
        self.create_subscription(SafetyBounds,topic('bounds'),self.safety,qos)
        self.subs=[self.create_subscription(PoseRedchoNeighbor,topic(f'cf{peer}/position_neighbor'),
            lambda msg,peer=peer:self.receive(peer,msg),qos) for peer in self.model.neighbors]
        self.create_timer(.2,self.ready)
        self.get_logger().info(f'{scenario.IMPLEMENTATION_VERSION}: cf{value}, pid={os.getpid()}, ONE position bank, ring={self.model.neighbors}')

    def ready(self):
        if (self.model.reference.cycle==0 and not self.failed and self.pub.get_subscription_count()==2
                and self.results.get_subscription_count()>=1 and self.prepared.get_subscription_count()==1):
            msg=RingAck();msg.cycle=0;msg.agent_id=self.agent_id;msg.neighbor_ids=list(self.model.neighbors)
            msg.valid=True;msg.detail=json.dumps(identity(os.getpid()));self.ready_pub.publish(msg)

    def begin(self,msg):
        if self.failed:return
        try:
            state=(np.asarray(msg.positions).reshape(3,6),np.asarray(msg.velocities).reshape(3,6),
                np.asarray(msg.rotations).reshape(3,3,6),np.asarray(msg.angular_velocities).reshape(3,6))
            estimate,bound=self.model.prepare(int(msg.cycle),float(msg.mission_time),state,
                np.asarray(msg.previous_accelerations).reshape(3,6))
            self.pending={k:v for k,v in self.pending.items() if k>=msg.cycle}
            self.bounds={k:v for k,v in self.bounds.items() if k>=msg.cycle}
            neighbor=PoseRedchoNeighbor();neighbor.cycle=msg.cycle;neighbor.agent_id=self.agent_id
            neighbor.mission_time=msg.mission_time;neighbor.displacement_estimate=estimate.tolist();self.pub.publish(neighbor)
            prepared=ControlPrepared();prepared.cycle=msg.cycle;prepared.agent_id=self.agent_id
            prepared.acceleration_bound=float(bound);self.prepared.publish(prepared);self.finish()
        except Exception as exc:self.fail(str(exc))

    def receive(self,peer,msg):
        if self.failed:return
        try:
            if msg.agent_id!=peer:raise ValueError('Neighbor/topic identity mismatch')
            cycle=int(msg.cycle);current=self.model.reference.cycle
            if cycle<current:return
            if cycle<1 or cycle>current+1:raise ValueError('Neighbor cycle mismatch')
            if not np.isfinite(msg.mission_time) or abs(msg.mission_time-(cycle-1)*self.model.dt)>1e-9:
                raise ValueError('Neighbor timestamp mismatch')
            value=np.asarray(msg.displacement_estimate,float)
            if value.shape!=(3,) or not np.isfinite(value).all():raise ValueError('Invalid position REDCHO packet')
            peers=self.pending.setdefault(cycle,{})
            if peer in peers:raise ValueError('Duplicate neighbor message')
            peers[peer]=value.copy();self.finish()
        except Exception as exc:self.fail(str(exc))

    def safety(self,msg):
        if self.failed:return
        try:
            if msg.cycle!=self.model.reference.cycle:raise ValueError('Safety bound cycle mismatch')
            if msg.cycle in self.bounds:raise ValueError('Duplicate safety bounds')
            self.bounds[msg.cycle]=np.asarray(msg.acceleration_bounds,float);self.finish()
        except Exception as exc:self.fail(str(exc))

    def finish(self):
        if self.model.finished:return
        cycle=self.model.reference.cycle;peers=self.pending.get(cycle,{})
        if cycle not in self.bounds or set(peers)!=set(self.model.neighbors):return
        d=self.model.finish(peers,self.bounds[cycle])
        msg=ControlResult();msg.cycle=cycle;msg.agent_id=self.agent_id;msg.valid=True
        msg.detail=json.dumps(identity(os.getpid()));msg.safe_acceleration=d['u_safe_hist'].tolist()
        msg.motor_forces=d['u_motors_hist'].tolist();msg.motor_limit=float(d['F_max_hist'])
        msg.diagnostics_json=json.dumps({k:np.asarray(v).tolist() for k,v in d.items()},allow_nan=False)
        self.results.publish(msg)

    def fail(self,error):
        self.failed=True;self.get_logger().error(error)
        msg=ControlResult();msg.cycle=self.model.reference.cycle;msg.agent_id=self.agent_id;msg.valid=False
        msg.detail=json.dumps(dict(**identity(os.getpid()),error=error));self.results.publish(msg)


def main(args=None):
    rclpy.init(args=args);node=ControlAgent()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
