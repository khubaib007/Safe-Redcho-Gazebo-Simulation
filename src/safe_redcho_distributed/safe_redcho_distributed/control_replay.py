# Release preparation: portable defaults; original control equations retained.
"""Verify full local controllers against recorded baseline physical states."""
import argparse
import json
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from safe_redcho_distributed_interfaces.msg import ControlFrame,ControlPrepared,SafetyBounds,ControlResult,RingAck
from safe_redcho_swarm.core.quadrotor import integrate_crazyflie_se3
from safe_redcho_swarm.core.config import Params
from .local_controller import initial_plant_state
from .ring_protocol import neighbors

DIAGNOSTIC_KEYS=['SoC_hist','V_batt_hist','v_safe_hist','raw_ref_hist','F_max_hist','u_max_hist',
 'model_mismatch_hist','disturbance_bound_hist','U_hist','Y_hist','U_levant_hist','Y_levant_hist',
 'svf_snap_hist','u_safe_hist','u_motors_hist','P_elec_hist','I_batt_hist','attitude_error_hist',
 'motor_utilization_hist','accel_utilization_hist','slew_utilization_hist','qp_correction_hist',
 'plant_agent_err_hist','qp_phase1_slack_hist','qp_violation_hist','qp_mode_hist',
 'row_actuator_margin_hist','recursive_feas_margin_hist']


class ControlReplay(Node):
    def __init__(self,data,steps,*,node_name="safe_redcho_control_replay",initial_message=None):
        super().__init__(node_name)
        self.data=data;self.steps=steps;self.current=0;self.completed=0;self.tolerance=1e-7
        self.ready=set();self.received=set();self.prepared={};self.pids={};self.bounds_sent=False
        self.counts={i:0 for i in range(1,7)};self.max_error={key:0. for key in DIAGNOSTIC_KEYS}
        self.h={key:np.zeros_like(data[key][...,:steps]) for key in DIAGNOSTIC_KEYS}
        self.state=initial_plant_state();self.params=Params()
        self.error=None;self.finished=False;self.start=time.perf_counter();self.deadline=self.start+30.
        qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE)
        self.pub=self.create_publisher(ControlFrame,'/safe_redcho_distributed/control_frame',qos)
        self.bound_pub=self.create_publisher(SafetyBounds,'/safe_redcho_distributed/safety_bounds',qos)
        self.subs=[]
        for i in range(1,7):
            self.subs.append(self.create_subscription(ControlResult,f'/cf{i}/control_result',lambda msg,i=i:self.receive(i,msg),qos))
            self.subs.append(self.create_subscription(ControlPrepared,f'/cf{i}/control_prepared',lambda msg,i=i:self.prepare(i,msg),qos))
        self.ready_sub=self.create_subscription(RingAck,'/safe_redcho_distributed/control_ready',self.readiness,qos)
        self.timer=self.create_timer(.005,self.advance)
        self.get_logger().info(initial_message or 'Waiting for six complete local controller processes; replay verification only')

    def readiness(self,msg):
        if self.finished:return
        try:
            i=int(msg.agent_id)
            if msg.cycle!=0 or not msg.valid or tuple(msg.neighbor_ids)!=neighbors(i):raise ValueError('Invalid readiness')
            pid=int(json.loads(msg.detail)['pid'])
            if i in self.pids and self.pids[i]!=pid:raise ValueError('Agent process changed')
            self.pids[i]=pid;self.ready.add(i)
        except Exception as exc:self.fail(str(exc))

    def prepare(self,i,msg):
        if self.finished:return
        try:
            if msg.agent_id!=i or msg.cycle!=self.current or i in self.prepared:raise ValueError('Wrong/duplicate prepared cycle or ID')
            value=float(msg.acceleration_bound)
            if not np.isfinite(value) or value<=0:raise ValueError('Invalid acceleration bound')
            self.prepared[i]=value
            if len(self.prepared)==6:
                bounds=SafetyBounds();bounds.cycle=self.current;bounds.acceleration_bounds=[self.prepared[i] for i in range(1,7)]
                self.bound_pub.publish(bounds);self.bounds_sent=True
        except Exception as exc:self.fail(str(exc))

    def receive(self,i,msg):
        if self.finished:return
        try:
            if msg.agent_id!=i:raise ValueError('Result/topic ID mismatch')
            detail=json.loads(msg.detail)
            if not msg.valid:raise ValueError(f'cf{i}: '+detail.get('error','local controller failed'))
            if int(detail['pid'])!=self.pids.get(i):raise ValueError('Agent process changed')
            if msg.cycle!=self.current or i in self.received or not self.bounds_sent:raise ValueError('Result cycle/order mismatch')
            d=json.loads(msg.diagnostics_json)
            if set(d)!=set(DIAGNOSTIC_KEYS):raise ValueError('Diagnostic field mismatch')
            if not np.array_equal(np.asarray(msg.safe_acceleration),d['u_safe_hist']) or not np.array_equal(np.asarray(msg.motor_forces),d['u_motors_hist']) or msg.motor_limit!=d['F_max_hist']:raise ValueError('Commands disagree with diagnostics')
            k=self.current-1
            for key in DIAGNOSTIC_KEYS:
                value=np.asarray(d[key],dtype=float);expected=self.data[key][...,i-1,k]
                if value.shape!=expected.shape or not np.isfinite(value).all():raise ValueError('Invalid diagnostic '+key)
                delta=float(np.max(np.abs(value-expected)));self.max_error[key]=max(self.max_error[key],delta)
                if delta>self.tolerance:raise ValueError(f'cf{i} t={self.data["time"][k]} {key}: difference {delta} exceeds {self.tolerance}')
                self.h[key][...,i-1,k]=value
            self.received.add(i);self.counts[i]+=1
        except Exception as exc:self.fail(str(exc))

    def fail(self,error):self.error=error;self.finished=True

    def advance(self):
        if self.finished:return
        if time.perf_counter()>self.deadline:
            self.fail(f'Timeout: cycle={self.current}, ready={sorted(self.ready)}, prepared={sorted(self.prepared)}, results={sorted(self.received)}');return
        if self.current==0:
            if len(self.ready)!=6 or self.pub.get_subscription_count()!=6 or self.bound_pub.get_subscription_count()!=6:return
            if len(set(self.pids.values()))!=6:self.fail('Expected six distinct processes');return
        else:
            if len(self.received)!=6:return
            self.completed=self.current
            if self.current%500==0:self.get_logger().info(f'Checked {self.current}/{self.steps} cycles; maximum difference={max(self.max_error.values()):.3e}')
            if self.current==self.steps:self.finished=True;return
            # Only reconstruct the baseline attitude/body-rate input from its
            # recorded motor commands. New commands are verified, not applied.
            k=self.current-1;p,v,R,w=self.state
            for i in range(6):
                p[:,i],v[:,i],R[:,:,i],w[:,i]=integrate_crazyflie_se3(p[:,i],v[:,i],R[:,:,i],w[:,i],self.data['u_motors_hist'][:,i,k],self.params,.01)
        self.current+=1;self.received.clear();self.prepared.clear();self.bounds_sent=False
        self.deadline=time.perf_counter()+5.;k=self.current-1
        msg=ControlFrame();msg.cycle=self.current;msg.mission_time=float(self.data['time'][k])
        msg.positions=self.data['p_hist'][...,k].ravel().tolist();msg.velocities=self.data['v_hist'][...,k].ravel().tolist()
        msg.rotations=self.state[2].ravel().tolist();msg.angular_velocities=self.state[3].ravel().tolist()
        previous=np.zeros((3,6)) if k==0 else self.data['u_safe_hist'][...,k-1]
        msg.previous_accelerations=previous.ravel().tolist();self.pub.publish(msg)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,default=Path('results/reference/simulation_data.npz'))
    parser.add_argument('--duration',type=float,default=40.)
    parser.add_argument('--output',type=Path,default=Path('results/control_replay'))
    args=parser.parse_args()
    if not np.isfinite(args.duration) or args.duration<0 or abs(args.duration/.01-round(args.duration/.01))>1e-8:parser.error('duration must be a nonnegative multiple of .01')
    steps=int(round(args.duration/.01))+1
    with np.load(args.reference,allow_pickle=False) as f:data={key:f[key] for key in ['time','p_hist','v_hist',*DIAGNOSTIC_KEYS]}
    if len(data['time'])<steps or not np.allclose(data['time'][:steps],np.arange(steps)*.01,rtol=0,atol=1e-9):parser.error('Reference must start at t=0, dt=.01 and cover duration')
    for key in ['p_hist','v_hist',*DIAGNOSTIC_KEYS]:
        a=data[key]
        if a.shape[-2:]!=(6,len(data['time'])) or not np.isfinite(a).all():parser.error('Invalid reference array '+key)
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=[]);node=ControlReplay(data,steps)
    try:
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:node.error='Interrupted'
    finally:
        success=node.finished and node.error is None and node.completed==steps
        report={'status':'PASS' if success else 'FAIL','reference':str(args.reference.resolve()),
                'requested_duration':args.duration,'completed_cycles':node.completed,'expected_cycles':steps,
                'agent_process_ids':node.pids,'results_per_agent':node.counts,
                'compared_quantities':len(DIAGNOSTIC_KEYS),'max_absolute_difference':node.max_error,
                'tolerance':node.tolerance,'ROS_DOMAIN_ID':os.environ.get('ROS_DOMAIN_ID','0'),
                'error':node.error,'wall_duration':time.perf_counter()-node.start,
                'scope':'Six complete local battery/SVF/REDCHO/QP/geometric controllers; physical-state replay, not closed-loop Gazebo'}
        (args.output/'control_status.json').write_text(json.dumps(report,indent=2))
        h={key:a[...,:node.completed] for key,a in node.h.items()};h['time']=data['time'][:node.completed]
        np.savez_compressed(args.output/'control_data.npz',**h)
        print(json.dumps(report,indent=2));node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if not success:raise RuntimeError('Full local controller replay failed; inspect control_status.json')


if __name__=='__main__':main()
