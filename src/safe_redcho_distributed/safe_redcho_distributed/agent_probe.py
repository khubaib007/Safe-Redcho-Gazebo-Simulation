"""One independent per-drone ring transport probe; sends no flight commands."""
import json
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from std_msgs.msg import UInt64
from safe_redcho_distributed_interfaces.msg import RingProbe,RingAck
from .ring_protocol import RingInbox,payload


class AgentProbe(Node):
    def __init__(self):
        super().__init__('safe_redcho_agent_probe')
        self.declare_parameter('agent_id',1)
        self.agent_id=int(self.get_parameter('agent_id').value)
        self.inbox=RingInbox(self.agent_id)
        self.failed=False
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=16,reliability=ReliabilityPolicy.RELIABLE)
        self.publisher=self.create_publisher(RingProbe,f'/cf{self.agent_id}/redcho_probe',qos)
        self.ack=self.create_publisher(RingAck,'/safe_redcho_distributed/probe_ack',qos)
        self.peers=[self.create_subscription(RingProbe,f'/cf{peer}/redcho_probe',
                    lambda msg,peer=peer:self.receive(peer,msg),qos) for peer in self.inbox.neighbors]
        self.tick=self.create_subscription(UInt64,'/safe_redcho_distributed/probe_tick',self.begin,qos)
        self.timer=self.create_timer(.2,self.ready)
        self.get_logger().info(f'agent={self.agent_id}, pid={os.getpid()}, ring neighbors={self.inbox.neighbors}')

    def respond(self,cycle,valid,error=''):
        msg=RingAck();msg.cycle=cycle;msg.agent_id=self.agent_id
        msg.neighbor_ids=list(self.inbox.neighbors);msg.valid=valid
        msg.detail=json.dumps({'pid':os.getpid(),'error':error})
        self.ack.publish(msg)

    def ready(self):
        if self.inbox.cycle==0 and not self.failed and self.publisher.get_subscription_count()==2:
            self.respond(0,True)

    def begin(self,msg):
        if self.failed:return
        try:
            self.inbox.begin(int(msg.data))
            outgoing=RingProbe();outgoing.cycle=int(msg.data);outgoing.agent_id=self.agent_id
            outgoing.payload=payload(self.agent_id,int(msg.data))
            self.publisher.publish(outgoing)
            self.finish()
        except Exception as exc:self.fail(str(exc))

    def receive(self,peer,msg):
        if self.failed:return
        try:
            if msg.agent_id!=peer:raise ValueError('Publisher/topic agent ID mismatch')
            self.inbox.receive(peer,int(msg.cycle),msg.payload)
            self.finish()
        except Exception as exc:self.fail(str(exc))

    def finish(self):
        if self.inbox.complete():self.respond(self.inbox.cycle,True)

    def fail(self,error):
        self.failed=True
        self.get_logger().error(error)
        self.respond(self.inbox.cycle,False,error)


def main(args=None):
    rclpy.init(args=args);node=AgentProbe()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
