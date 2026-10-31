"""Run 100 barrier-synchronized ring cycles without Gazebo or a controller."""
import argparse
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from std_msgs.msg import UInt64
from safe_redcho_distributed_interfaces.msg import RingAck
from .ring_protocol import neighbors


class RingCheck(Node):
    def __init__(self,cycles):
        super().__init__('safe_redcho_ring_check')
        self.cycles=cycles;self.current=0;self.completed=0
        self.ready=set();self.received=set();self.pids={};self.counts={i:0 for i in range(1,7)}
        self.start=time.perf_counter();self.deadline=self.start+30.
        self.finished=False;self.error=None
        qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=32,reliability=ReliabilityPolicy.RELIABLE)
        self.pub=self.create_publisher(UInt64,'/safe_redcho_distributed/probe_tick',qos)
        self.sub=self.create_subscription(RingAck,'/safe_redcho_distributed/probe_ack',self.receive,qos)
        self.timer=self.create_timer(.02,self.advance)
        self.get_logger().info('Waiting for six agent processes and twelve directed ring links')

    def receive(self,msg):
        if self.finished:return
        try:
            agent=int(msg.agent_id)
            if tuple(msg.neighbor_ids)!=neighbors(agent):raise ValueError('Incorrect ring neighbors')
            detail=json.loads(msg.detail)
            if not msg.valid:raise ValueError(f'cf{agent}: '+detail.get('error','agent failed'))
            pid=int(detail['pid'])
            if agent in self.pids and self.pids[agent]!=pid:raise ValueError('Agent restarted during test')
            self.pids[agent]=pid
            if msg.cycle==0:
                self.ready.add(agent);return
            if msg.cycle!=self.current:raise ValueError('Acknowledgment cycle mismatch')
            if agent in self.received:raise ValueError('Duplicate cycle acknowledgment')
            self.received.add(agent);self.counts[agent]+=1
        except Exception as exc:
            self.error=str(exc);self.finished=True

    def advance(self):
        if self.finished:return
        if time.perf_counter()>self.deadline:
            self.error=f'Timeout at cycle {self.current}; ready={sorted(self.ready)}; acknowledgments={sorted(self.received)}'
            self.finished=True;return
        if self.current==0:
            if len(self.ready)!=6 or self.pub.get_subscription_count()!=6:return
            if len(set(self.pids.values()))!=6:
                self.error='Expected six distinct agent process IDs';self.finished=True;return
        else:
            if len(self.received)!=6:return
            self.completed=self.current
            if self.current%25==0:self.get_logger().info(f'Completed {self.current}/{self.cycles} cycles')
            if self.current==self.cycles:
                self.finished=True;return
        self.current+=1;self.received.clear();self.deadline=time.perf_counter()+5.
        msg=UInt64();msg.data=self.current;self.pub.publish(msg)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cycles',type=int,default=100)
    parser.add_argument('--output',type=Path,default=Path.home()/'safe_redcho_distributed_ws/results/ring_check.json')
    args=parser.parse_args()
    if args.cycles<1:parser.error('cycles must be positive')
    rclpy.init(args=[]);node=RingCheck(args.cycles)
    try:
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:node.error='Interrupted'
    finally:
        success=node.finished and node.error is None and node.completed==args.cycles
        report={'status':'PASS' if success else 'FAIL','requested_cycles':args.cycles,
                'completed_cycles':node.completed,'agents':6,'directed_ring_links':12,
                'agent_process_ids':node.pids,'acknowledgments_per_agent':node.counts,
                'ROS_DOMAIN_ID':os.environ.get('ROS_DOMAIN_ID','0'),
                'wall_duration':time.perf_counter()-node.start,'error':node.error,
                'scope':'Ring transport and cycle barrier only; no controller equations or flight commands'}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if not success:raise RuntimeError('Ring check failed; see output JSON')


if __name__=='__main__':main()
