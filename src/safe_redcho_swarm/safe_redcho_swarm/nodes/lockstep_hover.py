"""Six-agent timing/actuation diagnostic, preceding the REDCHO mission."""
from pathlib import Path
import csv
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from safe_redcho_interfaces.srv import StepSwarm
from safe_redcho_swarm.core.config import Params
from safe_redcho_swarm.core.quadrotor import crazyflie_geometric_controller
from safe_redcho_swarm.nodes.lockstep_protocol import validate_reply


class LockstepHover(Node):
    def __init__(self):
        super().__init__('safe_redcho_lockstep_hover')
        self.declare_parameter('duration',15.0)
        self.declare_parameter('output_dir',str(Path.home()/'ros2_ws/results/safe_redcho_lockstep'))
        self.client = self.create_client(StepSwarm, '/safe_redcho/step')
        self.params = Params()
        self.targets = np.array([[-3.5-col*1.25,-3.5-row*1.25,1.]
                                 for row in range(2) for col in range(3)])

    def run(self):
        duration = float(self.get_parameter('duration').value)
        count = round(duration/.01)
        if count < 100 or abs(count*.01-duration) > 1e-9:
            raise ValueError('duration must be >=1 s and an integer multiple of .01 s')
        directory = Path(str(self.get_parameter('output_dir').value))
        directory.mkdir(parents=True, exist_ok=True)
        report = {'status':'RUNNING','requested_duration':duration,'agents':6}
        self.get_logger().info('Waiting for /safe_redcho/step; Gazebo must start paused.')
        wall0 = time.monotonic()
        while not self.client.wait_for_service(timeout_sec=1.):
            if not rclpy.ok() or time.monotonic()-wall0 > 30:
                raise RuntimeError('Lockstep service unavailable after 30 s')
        commanded = np.full((6,4),.0299*9.81/4)
        limits = np.full(6,.16)
        previous_ns = 0
        records = []
        fixed_thrust_ok = False
        wall0 = time.monotonic()
        try:
            with (directory/'hover.csv').open('w',newline='',encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow(['sequence','time','agent','x','y','z','vx','vy','vz',
                                 'attitude_error','f1','f2','f3','f4'])
                for sequence in range(1,count+1):
                    if not rclpy.ok():
                        raise RuntimeError('ROS shutdown during timing check')
                    # Deliberately hold physics for 250 ms once. The next reply
                    # must still advance exactly 10 ms, demonstrating that wall
                    # computation delays do not advance the simulated plant.
                    if sequence == 51:
                        self.get_logger().info('Pause probe: delaying next command by 250 ms wall time')
                        time.sleep(.25)
                    request = StepSwarm.Request()
                    request.sequence = sequence
                    request.forces = commanded.ravel().tolist()
                    request.limits = limits.tolist()
                    future = self.client.call_async(request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=12.)
                    if not future.done():
                        self.client.remove_pending_request(future)
                        raise RuntimeError('Step response timeout; stop and restart the launch')
                    reply = future.result()
                    ns,pos,vel,R,om = validate_reply(
                        reply,sequence,previous_ns,commanded,limits)
                    previous_ns = ns
                    elapsed = ns*1e-9
                    attitude = []
                    next_commands = []
                    for i in range(6):
                        accel = np.clip(np.array([1.5,1.5,3.])*(self.targets[i]-pos[i])
                                        -np.array([2.5,2.5,3.])*vel[i],-2.8,2.8)
                        motors,error = crazyflie_geometric_controller(
                            R[i],om[i],vel[i],accel,self.params,limits[i])
                        next_commands.append(motors)
                        attitude.append(error)
                        writer.writerow([sequence,elapsed,i+1,*pos[i],*vel[i],
                                         error,*np.asarray(reply.applied_forces).reshape(6,4)[i]])
                    commanded = (np.full((6,4),.0299*9.81/4)
                                 if sequence < 50 else np.array(next_commands))
                    report['final_altitudes'] = pos[:,2].tolist()
                    report['last_applied_motor_forces'] = np.asarray(reply.applied_forces).reshape(6,4).tolist()
                    distance = np.linalg.norm(pos[:,None,:]-pos[None,:,:],axis=-1)
                    min_pair = float(distance[np.triu_indices(6,1)].min())
                    records.append((elapsed,float(np.max(np.abs(pos[:,2]-1.))),
                                    float(np.max(np.linalg.norm(vel,axis=1))),
                                    float(np.max(np.linalg.norm(pos[:,:2]-self.targets[:,:2],axis=1))),
                                    min_pair))
                    if sequence == 50:
                        probe = np.array(records)
                        height = float(probe[:,1].max())
                        speed = float(probe[:,2].max())
                        fixed_thrust_ok = height < .0005 and speed < .002
                        report['fixed_thrust_probe'] = {
                            'pass':fixed_thrust_ok,'duration':.5,
                            'max_height_error':height,'max_speed':speed}
                        self.get_logger().info(
                            f'FIXED-THRUST PROBE {"PASS" if fixed_thrust_ok else "FAIL"}: '
                            f'height error={height:.6f}m, speed={speed:.6f}m/s')
                        if not fixed_thrust_ok:
                            raise RuntimeError('Fixed mg/4 hover probe failed; inspect physical wrench handling')
                    if sequence % 100 == 0:
                        file.flush()
                        self.get_logger().info(
                            f't={elapsed:.2f}s | six synchronized states | '
                            f'max height error={records[-1][1]:.6f}m | min pair={min_pair:.4f}m')
            data = np.array(records)
            tail = data[data[:,0] >= max(0.,duration-5.)]
            checks = {
                'all_step_intervals_exactly_10ms':True,
                'all_step_wrench_counts_exactly_10':True,
                'fixed_thrust_probe':fixed_thrust_ok,
                'all_six_stamps_equal':True,
                'all_applied_motor_batches_match':True,
                'pause_probe':True,
                'hover_height':bool(tail[:,1].max()<.02),
                'hover_speed':bool(tail[:,2].max()<.03),
                'hover_xy':bool(tail[:,3].max()<.02),
                'interagent_separation':bool(data[:,4].min()>1.20)}
            report.update(status='PASS' if all(checks.values()) else 'FAIL',
                completed_steps=count,simulated_duration=previous_ns*1e-9,
                wall_duration=time.monotonic()-wall0,
                max_tail_height_error=float(tail[:,1].max()),
                max_tail_speed=float(tail[:,2].max()),
                max_tail_xy_error=float(tail[:,3].max()),
                min_interagent_distance=float(data[:,4].min()),checks=checks)
        except BaseException as exc:
            report.update(status='FAIL',error=str(exc),completed_steps=len(records),
                          simulated_duration=previous_ns*1e-9)
            raise
        finally:
            (directory/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            self.get_logger().info('LOCKSTEP CHECK '+report['status']+'\n'+json.dumps(report,indent=2))
        if report['status'] != 'PASS':
            raise RuntimeError('Lockstep hover checks failed')
        self.get_logger().info('World is now paused. Results: '+str(directory))


def main(args=None):
    rclpy.init(args=args)
    node = LockstepHover()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
