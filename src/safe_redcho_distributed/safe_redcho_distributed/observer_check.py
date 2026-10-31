"""Replay recorded local references and verify six ROS observer processes."""
import argparse
import json
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from safe_redcho_distributed_interfaces.msg import ObserverInput,ObserverResult,RingAck
from .ring_protocol import neighbors

FIELDS={'svf_state':'U_hist','svf_estimate':'Y_hist',
        'levant_state':'U_levant_hist','levant_estimate':'Y_levant_hist','snap':'svf_snap_hist'}


class ObserverCheck(Node):
    def __init__(self,data,steps,tolerance):
        super().__init__('safe_redcho_observer_check')
        self.data=data;self.steps=steps;self.tolerance=tolerance
        self.cycle=0;self.completed=0;self.ready=set();self.received=set();self.pids={}
        self.counts={i:0 for i in range(1,7)};self.max_error={field:0. for field in FIELDS}
        self.error=None;self.finished=False;self.start=time.perf_counter();self.deadline=self.start+30.
        self.h={key:np.zeros_like(data[key][...,:steps]) for key in FIELDS.values()}
        qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
        self.pubs={i:self.create_publisher(ObserverInput,f'/cf{i}/observer_input',qos) for i in range(1,7)}
        self.subs=[self.create_subscription(ObserverResult,f'/cf{i}/observer_result',
                   lambda msg,i=i:self.receive(i,msg),qos) for i in range(1,7)]
        self.ready_sub=self.create_subscription(RingAck,'/safe_redcho_distributed/observer_ready',self.readiness,qos)
        self.timer=self.create_timer(.005,self.advance)
        self.get_logger().info('Waiting for six actual SVF/REDCHO agent processes')

    def readiness(self,msg):
        if self.finished:return
        try:
            i=int(msg.agent_id)
            if msg.cycle!=0 or not msg.valid or tuple(msg.neighbor_ids)!=neighbors(i):raise ValueError('Invalid readiness')
            pid=int(json.loads(msg.detail)['pid'])
            if i in self.pids and self.pids[i]!=pid:raise ValueError('Agent process changed')
            self.pids[i]=pid;self.ready.add(i)
        except Exception as exc:self.fail(str(exc))

    def receive(self,i,msg):
        if self.finished:return
        try:
            if msg.agent_id!=i:raise ValueError('Result/topic agent ID mismatch')
            detail=json.loads(msg.detail)
            if not msg.valid:raise ValueError(f'cf{i}: '+detail.get('error','observer failed'))
            if int(detail['pid'])!=self.pids.get(i):raise ValueError('Agent process changed')
            if msg.cycle!=self.cycle or i in self.received:raise ValueError('Wrong or duplicate result cycle')
            k=self.cycle-1
            for field,key in FIELDS.items():
                value=np.asarray(getattr(msg,field),dtype=float)
                shape=(3,) if field=='snap' else (3,3)
                value=value.reshape(shape)
                expected=self.data[key][...,i-1,k]
                if not np.isfinite(value).all():raise ValueError('Nonfinite observer output')
                delta=float(np.max(np.abs(value-expected)))
                self.max_error[field]=max(self.max_error[field],delta)
                if delta>self.tolerance:raise ValueError(f'cf{i}, cycle {self.cycle}, {field}: difference {delta} exceeds {self.tolerance}')
                self.h[key][...,i-1,k]=value
            self.received.add(i);self.counts[i]+=1
        except Exception as exc:self.fail(str(exc))

    def fail(self,error):self.error=error;self.finished=True

    def advance(self):
        if self.finished:return
        if time.perf_counter()>self.deadline:
            self.fail(f'Timeout: cycle={self.cycle}, ready={sorted(self.ready)}, results={sorted(self.received)}');return
        if self.cycle==0:
            if len(self.ready)!=6 or any(pub.get_subscription_count()!=1 for pub in self.pubs.values()):return
            if len(set(self.pids.values()))!=6:self.fail('Expected six distinct processes');return
        else:
            if len(self.received)!=6:return
            self.completed=self.cycle
            if self.cycle==self.steps:
                self.finished=True;return
            if self.cycle%500==0:self.get_logger().info(f'Checked {self.cycle}/{self.steps} cycles; maximum difference={max(self.max_error.values()):.3e}')
        self.cycle+=1;self.received.clear();self.deadline=time.perf_counter()+5.
        k=self.cycle-1
        for i,pub in self.pubs.items():
            msg=ObserverInput();msg.cycle=self.cycle;msg.agent_id=i
            msg.mission_time=float(self.data['time'][k]);msg.raw_reference=self.data['raw_ref_hist'][:,i-1,k].tolist()
            pub.publish(msg)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,default=Path.home()/'ros2_ws/results/safe_redcho_baseline/simulation_data.npz')
    parser.add_argument('--duration',type=float,default=40.)
    parser.add_argument('--output',type=Path,default=Path.home()/'safe_redcho_distributed_ws/results/observer_check')
    args=parser.parse_args()
    if not np.isfinite(args.duration) or args.duration<0 or abs(args.duration/.01-round(args.duration/.01))>1e-8:parser.error('duration must be a nonnegative multiple of .01 s')
    steps=int(round(args.duration/.01))+1
    with np.load(args.reference,allow_pickle=False) as f:data={k:f[k] for k in ['time','raw_ref_hist',*FIELDS.values()]}
    if len(data['time'])<steps or not np.allclose(data['time'][:steps],np.arange(steps)*.01,rtol=0,atol=1e-9):parser.error('reference must start at zero with .01 s samples and cover duration')
    for key in ['raw_ref_hist','svf_snap_hist']:
        if data[key].shape!=(3,6,len(data['time'])) or not np.isfinite(data[key]).all():parser.error('invalid '+key)
    for key in set(FIELDS.values())-{'svf_snap_hist'}:
        if data[key].shape!=(3,3,6,len(data['time'])) or not np.isfinite(data[key]).all():parser.error('invalid '+key)
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=[]);node=ObserverCheck(data,steps,1e-10)
    try:
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:node.error='Interrupted'
    finally:
        success=node.finished and node.error is None and node.completed==steps
        report={'status':'PASS' if success else 'FAIL','reference':str(args.reference.resolve()),
                'requested_duration':args.duration,'expected_cycles':steps,'completed_cycles':node.completed,
                'agent_process_ids':node.pids,'results_per_agent':node.counts,
                'max_absolute_difference':node.max_error,'tolerance':node.tolerance,
                'ROS_DOMAIN_ID':os.environ.get('ROS_DOMAIN_ID','0'),'error':node.error,
                'wall_duration':time.perf_counter()-node.start,
                'scope':'Actual local SVF and REDCHO, including Levant comparison, with recorded raw references; no battery integration, QP, motor control or Gazebo'}
        (args.output/'observer_status.json').write_text(json.dumps(report,indent=2))
        h={k:v[...,:node.completed] for k,v in node.h.items()};h['time']=data['time'][:node.completed]
        np.savez_compressed(args.output/'observer_data.npz',**h)
        print(json.dumps(report,indent=2));node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if not success:raise RuntimeError('Distributed observer validation failed; see observer_status.json')


if __name__=='__main__':main()
