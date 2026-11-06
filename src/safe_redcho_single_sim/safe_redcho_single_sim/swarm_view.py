"""Read-only RViz models, synchronized ring, traces and per-agent battery labels."""
import json
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker,MarkerArray
from safe_redcho_distributed_interfaces.msg import ControlResult

from . import scenario

COLORS=[(.15,.75,1.),(1.,.35,.15),(.25,.9,.45),(.85,.4,1.),(1.,.85,.2),(.1,.9,.85)]

class SwarmView(Node):
    def __init__(self):
        super().__init__('single_position_swarm_view')
        self.states={};self.battery={};self.traces={i:deque(maxlen=4001) for i in range(1,7)}
        self.last_stamp={};self.error_announced=False
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(MarkerArray,'/safe_redcho_single_sim/markers',qos)
        self.subs=[]
        for i in range(1,7):
            self.subs.append(self.create_subscription(Odometry,f'/cf{i}/state',lambda m,i=i:self.state(i,m),10))
            self.subs.append(self.create_subscription(ControlResult,f'/safe_redcho_single_sim/cf{i}/result',lambda m,i=i:self.result(i,m),10))
        self.timer=self.create_timer(.1,self.render)
        self.get_logger().info('Read-only swarm view: waiting for /cf1..6/state; RViz fixed frame world')

    def state(self,i,msg):
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if i in self.last_stamp and ns<self.last_stamp[i]:self.traces[i].clear();self.battery.pop(i,None)
        if ns!=self.last_stamp.get(i):
            p=msg.pose.pose.position;self.traces[i].append((p.x,p.y,p.z))
        self.last_stamp[i]=ns;self.states[i]=msg

    def result(self,i,msg):
        if not msg.valid:return
        try:
            d=json.loads(msg.diagnostics_json)
            self.battery[i]=(int(msg.cycle),float(d['SoC_hist']),float(d['V_batt_hist']))
        except (KeyError,ValueError,TypeError):pass

    def marker(self,ns,id,kind):
        m=Marker();m.header.frame_id='world';m.ns=ns;m.id=id;m.type=kind;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.color.a=1.;return m

    def render(self):
        array=MarkerArray()
        for i,state in sorted(self.states.items()):
            c=COLORS[i-1]
            m=self.marker('crazyflie',i,Marker.MESH_RESOURCE);m.pose=state.pose.pose
            m.mesh_resource='package://safe_redcho_scene/meshes/crazyflie_body.dae'
            m.mesh_use_embedded_materials=True;m.scale.x=m.scale.y=m.scale.z=1.;array.markers.append(m)
            # Authentic meshes positioned relative to body COM, as in upstream SDF.
            q=state.pose.pose.orientation;vector=np.array([q.x,q.y,q.z])
            for j,(x,y) in enumerate([(.031,-.031),(-.031,-.031),(-.031,.031),(.031,.031)],1):
                offset=np.array([x,y,.003575])
                rotated=offset+2*q.w*np.cross(vector,offset)+2*np.cross(vector,np.cross(vector,offset))
                prop=self.marker('propellers',4*(i-1)+j,Marker.MESH_RESOURCE)
                p=state.pose.pose.position;prop.pose.position=Point(x=float(p.x+rotated[0]),y=float(p.y+rotated[1]),z=float(p.z+rotated[2]))
                prop.pose.orientation=q
                prop.mesh_resource='package://safe_redcho_scene/meshes/'+('ccw_prop.dae' if j%2 else 'cw_prop.dae')
                prop.mesh_use_embedded_materials=True;prop.scale.x=prop.scale.y=prop.scale.z=1.;array.markers.append(prop)

            m=self.marker('paths',i,Marker.LINE_STRIP);m.scale.x=.018
            m.color.r,m.color.g,m.color.b=c;m.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in self.traces[i]]
            array.markers.append(m)
            m=self.marker('agent_labels',i,Marker.TEXT_VIEW_FACING);m.pose.position=Point(x=state.pose.pose.position.x,y=state.pose.pose.position.y,z=state.pose.pose.position.z+.20)
            m.scale.z=.14;m.color.r,m.color.g,m.color.b=c
            b=self.battery.get(i);m.text=f'cf{i}' if b is None else f'cf{i} | {100*b[1]:.1f}% | {b[2]:.2f} V'
            array.markers.append(m)
            # Locator sphere intentionally larger than the physical drone.
            m=self.marker('locators',i,Marker.SPHERE);m.pose.position=state.pose.pose.position
            m.scale.x=m.scale.y=m.scale.z=.10;m.color.r,m.color.g,m.color.b=c;m.color.a=.25
            array.markers.append(m)
        if len(self.states)==6 and len(set(self.last_stamp.values()))==1:
            m=self.marker('logical_ring',0,Marker.LINE_LIST);m.scale.x=.012
            m.color.r=.2;m.color.g=.85;m.color.b=1.;m.color.a=.6
            for i in range(1,7):
                for j in (i,i%6+1):
                    p=self.states[j].pose.pose.position;m.points.append(Point(x=p.x,y=p.y,z=p.z+.08))
            array.markers.append(m)
        else:
            m=self.marker('logical_ring',0,Marker.LINE_LIST);m.action=Marker.DELETE;array.markers.append(m)
        for i,(x,y,z) in enumerate(scenario.OBSTACLE_CENTERS.T):
            m=self.marker('obstacles',i,Marker.SPHERE);m.pose.position=Point(x=x,y=y,z=z)
            m.scale.x=m.scale.y=m.scale.z=2*scenario.OBSTACLE_RADIUS;m.color.r=1.;m.color.g=.35;m.color.b=.1;m.color.a=.6;array.markers.append(m)
        self.pub.publish(array)

def main(args=None):
    rclpy.init(args=args);node=SwarmView()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
