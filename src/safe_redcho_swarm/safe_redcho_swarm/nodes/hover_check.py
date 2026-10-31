"""Diagnostic hover controller; it is separate from the REDCHO mission controller."""
from pathlib import Path
import csv
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from safe_redcho_swarm.core.config import Params
from safe_redcho_swarm.core.quadrotor import crazyflie_geometric_controller


class HoverCheck(Node):
    def __init__(self):
        super().__init__('safe_redcho_hover_check')
        self.declare_parameter('target_height',1.0)
        self.declare_parameter('output_csv',str(Path.home()/'ros2_ws/results/safe_redcho_hover/hover.csv'))
        self.params=Params()
        self.target=np.array([0.,0.,float(self.get_parameter('target_height').value)])
        self.pub=self.create_publisher(Float64MultiArray,'/cf1/motor_command',1)
        self.sub=self.create_subscription(Odometry,'/cf1/state',self.on_state,qos_profile_sensor_data)
        self.t0=None;self.last_t=None;self.last_report=-1.;self.records=[];self.reported=False
        output=Path(str(self.get_parameter('output_csv').value));output.parent.mkdir(parents=True,exist_ok=True)
        self.file=output.open('w',newline='',encoding='utf-8');self.writer=csv.writer(self.file)
        self.writer.writerow(['time','elapsed','x','y','z','vx','vy','vz','attitude_error','f1','f2','f3','f4'])
        self.get_logger().info('Waiting for /cf1/state. Diagnostic target: z=1 m. Output: '+str(output))

    def on_state(self,msg):
        t=msg.header.stamp.sec+1e-9*msg.header.stamp.nanosec
        if self.last_t is not None and t<=self.last_t:return
        self.last_t=t
        if self.t0 is None:self.t0=t
        elapsed=t-self.t0
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        pos=np.array([p.x,p.y,p.z]);quat=np.array([q.x,q.y,q.z,q.w])
        if not np.all(np.isfinite(np.r_[pos,quat])) or np.linalg.norm(quat)<1e-9:return
        R=Rotation.from_quat(quat).as_matrix()
        vl=msg.twist.twist.linear;om=msg.twist.twist.angular
        vel=R@np.array([vl.x,vl.y,vl.z])
        Omega=np.array([om.x,om.y,om.z])
        # Additional position PD exists only for this interface diagnostic.
        # The geometric attitude controller and motor allocation are unchanged.
        accel=np.array([1.5,1.5,3.0])*(self.target-pos)-np.array([2.5,2.5,3.0])*vel
        accel=np.clip(accel,-2.8,2.8)
        motors,attitude_error=crazyflie_geometric_controller(R,Omega,vel,accel,self.params,.16)
        command=Float64MultiArray();command.data=[*motors.tolist(),.16];self.pub.publish(command)
        self.writer.writerow([t,elapsed,*pos,*vel,attitude_error,*motors])
        self.records.append((elapsed,float(pos[2]),float(np.linalg.norm(vel)),float(np.linalg.norm(pos[:2])),float(attitude_error)))
        if elapsed-self.last_report>=1.:
            self.file.flush();self.last_report=elapsed
            self.get_logger().info(f'elapsed={elapsed:.1f} s | z={pos[2]:.4f} m | vz={vel[2]:.4f} m/s | ||e_R||={attitude_error:.3e} | F={motors.sum():.5f} N')
        if elapsed>=15. and not self.reported:
            tail=np.array([r for r in self.records if r[0]>=elapsed-5.])
            max_z_error=float(np.max(np.abs(tail[:,1]-self.target[2])))
            max_speed=float(tail[:,2].max());max_xy=float(tail[:,3].max())
            ok=max_z_error<.02 and max_speed<.03 and max_xy<.02
            self.get_logger().info(f'HOVER CHECK {"PASS" if ok else "FAIL"}: last 5 s max height error={max_z_error:.5f} m, max speed={max_speed:.5f} m/s, max xy displacement={max_xy:.5f} m. Continuing hover until launch is stopped.')
            self.reported=True

    def destroy_node(self):
        if hasattr(self,'file') and not self.file.closed:self.file.flush();self.file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node=HoverCheck()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
