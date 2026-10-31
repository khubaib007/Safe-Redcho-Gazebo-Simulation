"""One actual SVF/REDCHO observer process; no plant or motor commands."""
import json
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from safe_redcho_distributed_interfaces.msg import ObserverInput,ObserverNeighbor,ObserverResult,RingAck
from .local_observer import LocalObserver


class ObserverAgent(Node):
    def __init__(self):
        super().__init__('safe_redcho_observer_agent')
        self.declare_parameter('agent_id',1)
        self.agent_id=int(self.get_parameter('agent_id').value);self.model=LocalObserver(self.agent_id)
        self.pending={};self.failed=False
        qos=QoSProfile(depth=16,reliability=ReliabilityPolicy.RELIABLE)
        self.pub=self.create_publisher(ObserverNeighbor,f'/cf{self.agent_id}/redcho_neighbor',qos)
        self.results=self.create_publisher(ObserverResult,f'/cf{self.agent_id}/observer_result',qos)
        self.ready_pub=self.create_publisher(RingAck,'/safe_redcho_distributed/observer_ready',qos)
        self.input=self.create_subscription(ObserverInput,f'/cf{self.agent_id}/observer_input',self.begin,qos)
        self.subs=[self.create_subscription(ObserverNeighbor,f'/cf{peer}/redcho_neighbor',
                   lambda msg,peer=peer:self.receive(peer,msg),qos) for peer in self.model.neighbors]
        self.timer=self.create_timer(.2,self.ready)
        self.get_logger().info(f'agent={self.agent_id}, pid={os.getpid()}, actual SVF/REDCHO neighbors={self.model.neighbors}')

    def ready(self):
        if self.model.cycle==0 and not self.failed and self.pub.get_subscription_count()==2 and self.results.get_subscription_count()==1:
            msg=RingAck();msg.cycle=0;msg.agent_id=self.agent_id;msg.neighbor_ids=list(self.model.neighbors)
            msg.valid=True;msg.detail=json.dumps({'pid':os.getpid()});self.ready_pub.publish(msg)

    def begin(self,msg):
        if self.failed:return
        try:
            if msg.agent_id!=self.agent_id:raise ValueError('Local input agent ID mismatch')
            if abs(msg.mission_time-(msg.cycle-1)*.01)>1e-9:raise ValueError('Mission time/cycle mismatch')
            a,b=self.model.prepare(int(msg.cycle),msg.raw_reference)
            self.pending={k:v for k,v in self.pending.items() if k>=self.model.cycle}
            outgoing=ObserverNeighbor();outgoing.cycle=msg.cycle;outgoing.agent_id=self.agent_id
            outgoing.svf_velocity_estimate=a.tolist();outgoing.levant_velocity_estimate=b.tolist()
            self.pub.publish(outgoing);self.finish()
        except Exception as exc:self.fail(str(exc))

    def receive(self,peer,msg):
        if self.failed:return
        try:
            if msg.agent_id!=peer:raise ValueError('Neighbor/topic identity mismatch')
            cycle=int(msg.cycle)
            if cycle<self.model.cycle:return
            if cycle<1 or cycle>self.model.cycle+1:raise ValueError('Neighbor cycle mismatch')
            a=np.asarray(msg.svf_velocity_estimate);b=np.asarray(msg.levant_velocity_estimate)
            if not np.isfinite(np.r_[a,b]).all():raise ValueError('Nonfinite neighbor estimate')
            peers=self.pending.setdefault(cycle,{})
            if peer in peers:raise ValueError('Duplicate neighbor message')
            peers[peer]=(a,b);self.finish()
        except Exception as exc:self.fail(str(exc))

    def finish(self):
        if self.model.finished:return
        peers=self.pending.get(self.model.cycle,{})
        if set(peers)!=set(self.model.neighbors):return
        output=self.model.finish(peers)
        msg=ObserverResult();msg.cycle=self.model.cycle;msg.agent_id=self.agent_id;msg.valid=True
        msg.detail=json.dumps({'pid':os.getpid()})
        for key,value in output.items():setattr(msg,key,value.ravel().tolist())
        self.results.publish(msg)

    def fail(self,error):
        self.failed=True;self.get_logger().error(error)
        msg=ObserverResult();msg.cycle=self.model.cycle;msg.agent_id=self.agent_id;msg.valid=False
        msg.detail=json.dumps({'pid':os.getpid(),'error':error});self.results.publish(msg)


def main(args=None):
    rclpy.init(args=args);node=ObserverAgent()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
