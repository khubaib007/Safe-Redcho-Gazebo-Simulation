"""Battery/SVF/REDCHO/HOCBF/geometric controller with measured Gazebo feedback."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from safe_redcho_interfaces.srv import StepSwarm
from safe_redcho_swarm.core.mission_controller import MissionController, baseline_initial_state
from safe_redcho_swarm.nodes.lockstep_protocol import validate_reply
from safe_redcho_swarm.nodes.baseline import performance_summary


def controller_state(decoded):
    _,p,v,R,w=decoded
    return p.T,v.T,R.transpose(1,2,0),w.T


class MissionNode(Node):
    def __init__(self):
        super().__init__('safe_redcho_mission')
        self.client=self.create_client(StepSwarm,'/safe_redcho/step')

    def request(self,sequence,forces,limits):
        req=StepSwarm.Request()
        req.sequence=sequence
        req.forces=np.asarray(forces,dtype=float).ravel().tolist()
        req.limits=np.asarray(limits,dtype=float).ravel().tolist()
        future=self.client.call_async(req)
        rclpy.spin_until_future_complete(self,future,timeout_sec=12.)
        if not future.done():
            raise RuntimeError('Step service timed out; restart the world before retrying')
        return future.result()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=40.)
    parser.add_argument('--output',type=Path,default=Path.home()/'ros2_ws/results/safe_redcho_gazebo')
    parser.add_argument('--no-plots',action='store_true')
    args=parser.parse_args()
    if not np.isfinite(args.duration) or args.duration<0 or abs(args.duration/.01-round(args.duration/.01))>1e-8:
        parser.error('duration must be a nonnegative multiple of 0.01 seconds')
    args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=[])
    node=MissionNode(); controller=None; accepted=0; started=time.perf_counter()
    report={'status':'FAIL','requested_duration':args.duration,'accepted_motor_batches':0}
    try:
        node.get_logger().info('Waiting for fresh, paused mission_world (restart it for every run)')
        if not node.client.wait_for_service(timeout_sec=30.):
            raise RuntimeError('/safe_redcho/step unavailable')
        initial_reply=node.request(0,[],[])
        decoded=validate_reply(initial_reply,0,0,np.zeros((6,4)),np.full(6,.16),
                               expected_steps=1,expected_wrenches=0,expected_ns=1_000_000)
        epoch=previous_ns=decoded[0]
        initial=controller_state(decoded); desired=baseline_initial_state()
        departure={'position':float(np.max(np.linalg.norm(initial[0]-desired[0],axis=0))),
                   'velocity':float(np.max(np.linalg.norm(initial[1]-desired[1],axis=0))),
                   'rotation':float(np.max(np.abs(initial[2]-desired[2]))),
                   'angular_velocity':float(np.max(np.linalg.norm(initial[3],axis=0)))}
        report['initial_state_departure']=departure
        if departure['position']>.002 or departure['velocity']>.012 or departure['rotation']>1e-4 or departure['angular_velocity']>.01:
            raise RuntimeError(f'Unexpected initialized plant state: {departure}')
        controller=MissionController(args.duration,initial_state=initial,progress=True)
        command=next(controller.stream)
        while True:
            forces=command['motors'].T
            reply=node.request(accepted+1,forces,command['limits'])
            decoded=validate_reply(reply,accepted+1,previous_ns,forces,command['limits'])
            previous_ns=decoded[0];accepted+=1
            actual_state=controller_state(decoded)
            measured_accel=(actual_state[1]-command['state'][1])/.01
            report['last_batch_diagnostics']={
                'sequence':accepted,'controller_time':command['time'],
                'velocity_before_world':command['state'][1].T.tolist(),
                'velocity_after_world':actual_state[1].T.tolist(),
                'requested_acceleration_world':command['accel'].T.tolist(),
                'measured_acceleration_world':measured_accel.T.tolist(),
                'residual_per_agent':np.linalg.norm(measured_accel-command['accel'],axis=0).tolist(),
                'motor_forces':forces.tolist()}
            if accepted==1:
                report['first_batch_diagnostics']=report['last_batch_diagnostics'].copy()
                node.get_logger().info('First motor-batch residuals (m/s²): '+
                    str(report['first_batch_diagnostics']['residual_per_agent']))
            try:
                command=controller.stream.send(actual_state)
            except StopIteration as finished:
                h,meta=finished.value
                break
        meta.update(initialization_physics_steps=1,initialization_duration=.001,
                    initial_state_departure=departure,mission_epoch_ns=epoch,
                    final_sim_stamp_ns=previous_ns,accepted_motor_batches=accepted)
        summary=performance_summary(h,meta)
        report.update(status='PASS' if all(summary['checks'].values()) else 'FAIL',
                      checks=summary['checks'],accepted_motor_batches=accepted,
                      mission_plant_advance=(previous_ns-epoch)/1e9,
                      state_log_end_time=float(h['time'][-1]))
        np.savez_compressed(args.output/'simulation_data.npz',**h)
        (args.output/'metadata.json').write_text(json.dumps(meta,indent=2))
        (args.output/'performance_summary.json').write_text(json.dumps(summary,indent=2))
        if not args.no_plots:
            import matplotlib
            matplotlib.use('Agg')
            from safe_redcho_swarm.visualization.figures import make_plots
            make_plots(h,meta,args.output)
    except Exception as exc:
        report.update(error=str(exc),accepted_motor_batches=accepted)
        if controller is not None and hasattr(controller,'time'):
            np.savez_compressed(args.output/'partial_simulation_data.npz',**controller.snapshot())
            (args.output/'metadata.json').write_text(json.dumps(controller.metadata,indent=2))
    finally:
        report['wall_duration']=time.perf_counter()-started
        (args.output/'mission_status.json').write_text(json.dumps(report,indent=2))
        node.get_logger().info(json.dumps(report,indent=2))
        node.destroy_node();rclpy.shutdown()
    if report['status']!='PASS':
        raise RuntimeError(f"Mission failed: inspect {args.output/'mission_status.json'}; restart world before retrying")
    print(f'Outputs saved to {args.output.resolve()}; world remains paused')


if __name__=='__main__':
    main()
