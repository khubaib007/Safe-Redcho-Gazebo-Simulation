"""Transport barriers only: agents own all reference/control/battery computations."""
import json
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from safe_redcho_distributed_interfaces.msg import ControlFrame,ControlPrepared,SafetyBounds,ControlResult,RingAck
from .history import allocate_history
from .protocol import neighbors,topic,parse_identity


class ControlCoordinator(Node):
    def __init__(self,duration):
        super().__init__('single_position_gazebo_coordinator')
        self.h=allocate_history(duration);self.steps=len(self.h['time'])
        self.current=0;self.completed=0;self.ready=set();self.received=set();self.prepared={}
        self.pids={};self.bounds_sent=False;self.counts={i:0 for i in range(1,7)}
        self.error=None;self.finished=False;self.start=time.perf_counter();self.deadline=self.start+45.
        qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
        self.pub=self.create_publisher(ControlFrame,topic('frame'),qos)
        self.bound_pub=self.create_publisher(SafetyBounds,topic('bounds'),qos)
        self.subs=[]
        for i in range(1,7):
            self.subs.append(self.create_subscription(ControlResult,topic(f'cf{i}/result'),lambda msg,i=i:self.receive(i,msg),qos))
            self.subs.append(self.create_subscription(ControlPrepared,topic(f'cf{i}/prepared'),lambda msg,i=i:self.prepare(i,msg),qos))
        self.create_subscription(RingAck,topic('ready'),self.readiness,qos)
        self.create_timer(.005,self.advance)

    def readiness(self,msg):
        if self.finished:return
        try:
            i=int(msg.agent_id)
            if msg.cycle!=0 or not msg.valid or tuple(msg.neighbor_ids)!=neighbors(i):raise ValueError('Invalid readiness')
            d=parse_identity(msg.detail);pid=d['pid']
            if i in self.pids and self.pids[i]!=pid:raise ValueError('Agent process changed')
            self.pids[i]=pid;self.ready.add(i)
        except Exception as exc:self.fail(str(exc))

    def prepare(self,i,msg):
        if self.finished:return
        try:
            if (getattr(self,'phase',None)!='control' or msg.agent_id!=i or msg.cycle!=self.current or i in self.prepared):
                raise ValueError('Wrong/duplicate prepared cycle or ID')
            value=float(msg.acceleration_bound)
            if not np.isfinite(value) or value<=0:raise ValueError('Invalid acceleration bound')
            self.prepared[i]=value
            if len(self.prepared)==6:
                bounds=SafetyBounds();bounds.cycle=self.current
                bounds.acceleration_bounds=[self.prepared[j] for j in range(1,7)]
                self.bound_pub.publish(bounds);self.bounds_sent=True
        except Exception as exc:self.fail(str(exc))

    def fail(self,error):
        self.error=error;self.finished=True;self.get_logger().error(error)
