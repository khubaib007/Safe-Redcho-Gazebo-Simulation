# Release preparation: portable defaults; original control equations retained.
"""Advance Gazebo with six local command outputs and feed measured states back."""
import argparse
import json
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from safe_redcho_interfaces.srv import StepSwarm
from safe_redcho_distributed_interfaces.msg import ControlFrame
from safe_redcho_swarm.nodes.lockstep_protocol import validate_reply
from .mission_history import scenario_performance_summary as performance_summary
from . import scenario
from .control_replay import ControlReplay, DIAGNOSTIC_KEYS
from .local_controller import initial_plant_state
from .mission_history import allocate_history, mission_metadata, record_global_diagnostics


def controller_state(decoded):
    _,p,v,R,w=decoded
    return p.T.copy(),v.T.copy(),R.transpose(1,2,0).copy(),w.T.copy()


class GazeboMission(ControlReplay):
    """Reuse transport barriers only; override replay comparison and plant advance."""
    def __init__(self,duration):
        h=allocate_history(duration)
        super().__init__(h,len(h['time']),node_name='safe_redcho_distributed_gazebo',
            initial_message=f'{scenario.IMPLEMENTATION_VERSION}: waiting for six fresh control agents and a fresh, paused mission_world')
        self.h=h;self.metadata=mission_metadata(duration)
        self.client=self.create_client(StepSwarm,'/safe_redcho/step')
        self.phase='waiting';self.future=None
        self.previous=np.zeros((3,6));self.previous_ns=0;self.epoch=None
        self.accepted=0;self.departure=None;self.batch=None

    def receive(self,i,msg):
        if self.finished:return
        try:
            if msg.agent_id!=i:raise ValueError('Result/topic ID mismatch')
            detail=json.loads(msg.detail)
            if not msg.valid:raise ValueError(f'cf{i}: '+detail.get('error','local controller failed'))
            if int(detail['pid'])!=self.pids.get(i):raise ValueError('Agent process changed')
            if self.phase!='control' or msg.cycle!=self.current or i in self.received or not self.bounds_sent:
                raise ValueError('Result cycle/order mismatch')
            d=json.loads(msg.diagnostics_json)
            if set(d)!=set(DIAGNOSTIC_KEYS):raise ValueError('Diagnostic field mismatch')
            if (not np.array_equal(np.asarray(msg.safe_acceleration),d['u_safe_hist']) or
                not np.array_equal(np.asarray(msg.motor_forces),d['u_motors_hist']) or msg.motor_limit!=d['F_max_hist']):
                raise ValueError('Commands disagree with diagnostics')
            k=self.current-1
            for key in DIAGNOSTIC_KEYS:
                value=np.asarray(d[key],dtype=float)
                if value.shape!=self.h[key][...,i-1,k].shape or not np.isfinite(value).all():
                    raise ValueError('Invalid diagnostic '+key)
                self.h[key][...,i-1,k]=value
            motors=np.asarray(msg.motor_forces)
            if msg.motor_limit<=0 or np.any(motors<0) or np.any(motors>msg.motor_limit+1e-12):
                raise ValueError('Motor command outside local authority')
            if abs(d['u_max_hist']-self.prepared[i])>1e-12:raise ValueError('Safety bound changed within cycle')
            self.received.add(i);self.counts[i]+=1
        except Exception as exc:self.fail(str(exc))

    def request_step(self,sequence,forces,limits,phase):
        req=StepSwarm.Request();req.sequence=sequence
        req.forces=np.asarray(forces,dtype=float).ravel().tolist()
        req.limits=np.asarray(limits,dtype=float).ravel().tolist()
        self.future=self.client.call_async(req);self.phase=phase
        self.deadline=time.perf_counter()+12.

    def publish_frame(self):
        self.current+=1;self.received.clear();self.prepared.clear();self.bounds_sent=False
        self.phase='control';self.deadline=time.perf_counter()+10.
        msg=ControlFrame();msg.cycle=self.current;msg.mission_time=float(self.h['time'][self.current-1])
        p,v,R,w=self.state
        msg.positions=p.ravel().tolist();msg.velocities=v.ravel().tolist()
        msg.rotations=R.ravel().tolist();msg.angular_velocities=w.ravel().tolist()
        msg.previous_accelerations=self.previous.ravel().tolist();self.pub.publish(msg)

    def advance(self):
        if self.finished:return
        try:
            if time.perf_counter()>self.deadline:
                raise RuntimeError(f'Timeout: phase={self.phase}, cycle={self.current}, ready={sorted(self.ready)}, results={sorted(self.received)}')
            if self.phase=='waiting':
                if (len(self.ready)!=6 or self.pub.get_subscription_count()!=6 or
                    self.bound_pub.get_subscription_count()!=6 or not self.client.service_is_ready()):return
                if len(set(self.pids.values()))!=6:raise RuntimeError('Expected six distinct agent processes')
                self.request_step(0,[],[],'bootstrap');return
            if self.phase in ('bootstrap','step'):
                if not self.future.done():return
                reply=self.future.result()
                if self.phase=='bootstrap':
                    decoded=validate_reply(reply,0,0,np.zeros((6,4)),np.full(6,.16),
                        expected_steps=1,expected_wrenches=0,expected_ns=1_000_000)
                    self.state=controller_state(decoded);target=initial_plant_state()
                    self.departure=dict(position=float(np.max(np.linalg.norm(self.state[0]-target[0],axis=0))),
                        velocity=float(np.max(np.linalg.norm(self.state[1]-target[1],axis=0))),
                        rotation=float(np.max(np.abs(self.state[2]-target[2]))),
                        angular_velocity=float(np.max(np.linalg.norm(self.state[3]-target[3],axis=0))))
                    if any(self.departure[key]>bound for key,bound in
                           [('position',.002),('velocity',.012),('rotation',1e-4),('angular_velocity',.01)]):
                        raise RuntimeError(f'Unexpected bootstrap state: {self.departure}')
                    self.epoch=self.previous_ns=decoded[0]
                    self.publish_frame();return
                forces,limits=self.batch
                decoded=validate_reply(reply,self.current,self.previous_ns,forces,limits)
                self.previous_ns=decoded[0];self.state=controller_state(decoded)
                self.accepted+=1;self.completed=self.current
                self.previous=self.h['u_safe_hist'][...,self.current-1].copy()
                if self.completed==self.steps:self.finished=True;return
                self.publish_frame();return
            if self.phase=='control' and len(self.received)==6:
                k=self.current-1;record_global_diagnostics(self.h,k,self.state)
                if k%500==0:
                    self.get_logger().info(f't={self.h["time"][k]:.2f}s | phase={scenario.phase(self.h["time"][k])} | six local commands | min pair={self.h["min_dist_hist"][k]:.4f}m')
                self.batch=(self.h['u_motors_hist'][...,k].T.copy(),self.h['F_max_hist'][:,k].copy())
                self.request_step(self.current,*self.batch,'step')
        except Exception as exc:self.fail(str(exc))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=2.)
    parser.add_argument('--no-plots',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('results/distributed_gazebo'))
    args=parser.parse_args()
    if not np.isfinite(args.duration) or args.duration<0 or abs(args.duration/.01-round(args.duration/.01))>1e-8:
        parser.error('duration must be a nonnegative multiple of .01')
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=[]);node=GazeboMission(args.duration);summary=None
    try:
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.05)
        if not node.finished:node.fail('ROS shutdown before mission completion')
    except KeyboardInterrupt:node.fail('Interrupted; restart world and agents')
    except Exception as exc:node.fail(str(exc))
    success=node.finished and node.error is None and node.completed==node.steps
    h={key:a[...,:node.completed] for key,a in node.h.items()}
    meta=node.metadata | dict(initialization_physics_steps=1,initialization_duration=.001,
        initial_state_departure=node.departure,mission_epoch_ns=node.epoch,
        final_sim_stamp_ns=node.previous_ns,accepted_motor_batches=node.accepted)
    try:
        if success:
            summary=performance_summary(h,meta)
            success=all(summary['checks'].values())
            if not success:node.error='Performance checks failed'
            (args.output/'performance_summary.json').write_text(json.dumps(summary,indent=2))
        np.savez_compressed(args.output/('simulation_data.npz' if success else 'partial_simulation_data.npz'),**h)
        (args.output/'metadata.json').write_text(json.dumps(meta,indent=2))
        if success and not args.no_plots:
            import matplotlib
            matplotlib.use('Agg')
            from .scenario_figures import make_plots
            make_plots(h,meta,args.output)
    except Exception as exc:success=False;node.error=f'Output/plot error: {exc}'
    report=dict(status='PASS' if success else 'FAIL',requested_duration=args.duration,
        completed_cycles=node.completed,expected_cycles=node.steps,accepted_motor_batches=node.accepted,
        agent_process_ids=node.pids,results_per_agent=node.counts,agents=6,directed_ring_links=12,
        initial_state_departure=node.departure,
        mission_plant_advance=(node.previous_ns-node.epoch)/1e9 if node.epoch is not None else 0.,
        state_log_end_time=float(h['time'][-1]) if node.completed else None,
        checks=summary['checks'] if summary else {},error=node.error,
        ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID','0'),wall_duration=time.perf_counter()-node.start,
        scope='Closed-loop Gazebo; six local controllers with velocity REDCHO, position/height REDCHO and HOCBF-QP; coordinator handles transport, stepping and diagnostics')
    try:
        (args.output/'mission_status.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if not success:raise RuntimeError('Distributed Gazebo mission failed; inspect mission_status.json and restart world AND agents')
    print(f'Outputs saved to {args.output.resolve()}; world remains paused')


if __name__=='__main__':main()
