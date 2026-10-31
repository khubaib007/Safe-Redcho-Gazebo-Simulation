"""Run the actual agent callbacks with ROS transport stubs; no native ROS claim."""
import sys
import types
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class Message:
    pass


class Publisher:
    def __init__(self, topic):
        self.topic, self.messages = topic, []
    def publish(self, msg):
        self.messages.append(msg)
    def get_subscription_count(self):
        return 2 if self.topic.endswith(('control_redcho_neighbor', 'control_pose_redcho_neighbor')) else 1


class Node:
    def __init__(self, *args):
        pass
    def declare_parameter(self, *args):
        pass
    def get_parameter(self, name):
        return types.SimpleNamespace(value=1)
    def create_publisher(self, cls, topic, qos):
        return Publisher(topic)
    def create_subscription(self, *args):
        return args
    def create_timer(self, *args):
        pass
    def get_logger(self):
        return types.SimpleNamespace(info=lambda x: None, error=lambda x: None)


def module(name, **fields):
    m = types.ModuleType(name)
    m.__dict__.update(fields)
    sys.modules[name] = m


module('rclpy')
module('rclpy.node', Node=Node)
module('rclpy.qos', QoSProfile=lambda **kw: None,
       ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1))
module('safe_redcho_distributed_interfaces')
module('safe_redcho_distributed_interfaces.msg', **{name: Message for name in (
    'ControlFrame', 'ControlPrepared', 'SafetyBounds', 'ControlResult',
    'ObserverNeighbor', 'RingAck', 'PoseRedchoNeighbor')})
from safe_redcho_distributed.control_agent import ControlAgent
from safe_redcho_distributed.local_controller import initial_plant_state


def formation(peer=2, cycle=1, time=0.):
    m = Message()
    m.agent_id, m.cycle, m.mission_time = peer, cycle, time
    state = initial_plant_state()
    m.displacement_estimate = [0., 0., 0.]
    return m


def run():
    n = ControlAgent()
    n.ready()
    assert len(n.ready_pub.messages) == 1
    n.receive_formation(2, formation())  # Valid peer may arrive before own frame.
    state = initial_plant_state()
    frame = Message()
    frame.cycle, frame.mission_time = 1, 0.
    frame.positions, frame.velocities = state[0].ravel(), state[1].ravel()
    frame.rotations, frame.angular_velocities = state[2].ravel(), state[3].ravel()
    frame.previous_accelerations = np.zeros(18)
    n.begin(frame)
    assert not n.failed and len(n.formation_pub.messages) == 1
    bounds = Message()
    bounds.cycle, bounds.acceleration_bounds = 1, [n.model.bound]*6
    n.safety(bounds)
    for peer in (6, 2):
        m = Message()
        m.agent_id, m.cycle = peer, 1
        m.svf_velocity_estimate, m.levant_velocity_estimate = np.zeros(3), np.zeros(3)
        n.receive(peer, m)
    assert not n.results.messages, 'Command emitted without both formation peers'
    n.receive_formation(6, formation(peer=6))
    assert not n.failed and len(n.results.messages) == 1 and n.results.messages[0].valid
    bad = ControlAgent()
    bad.receive_formation(2, formation(time=.01))
    assert bad.failed and not bad.results.messages[0].valid
    duplicate = ControlAgent()
    duplicate.receive_formation(2, formation())
    duplicate.receive_formation(2, formation())
    assert duplicate.failed
    future = ControlAgent()
    future.receive_formation(2, formation(cycle=3, time=.02))
    assert future.failed
    print('PASS: readiness, early peer delivery, no command before all barriers, '
          'timestamp/duplicate/future rejection and latched failure')


if __name__ == '__main__':
    run()
