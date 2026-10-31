> Historical delivery note. Use the root README and current docs/ guides for this release.

# Step 3: single Gazebo body and state bridge

This is an incremental update to the Step 2 package. Extract it into ~/ros2_ws/src; it replaces setup.py and package.xml and adds a world, launch file, and bridge configuration. It does not replace the numerical controller modules or baseline executable.

The model is a simplified Crazyflie rigid body with the source mass (0.0299 kg), diagonal inertia (1.395e-5, 1.395e-5, 2.173e-5 kg m²), four decorative rotors, and a single collision box. Rotor visuals are not actuators. The ground top is z=0; the body starts at z=1 and falls to the ground with no motor forces. Physics step is 0.001 s. This is a connectivity/gravity check, not a flight or controller validation.

The world name is safe_redcho_single. The Gazebo odometry publisher is configured for three dimensions and a target 100 Hz publication rate. The ROS bridge exposes /cf1/odom as nav_msgs/msg/Odometry and /clock as rosgraph_msgs/msg/Clock. Clock is sourced from the world-qualified Gazebo topic to avoid ambiguous Gazebo /clock discovery. ROS state header frame is world and child frame cf1/base_link. Actual coordinate and twist conventions will be verified before controller integration; frame labels alone are not evidence of velocity-coordinate correctness. Gazebo OdometryPublisher is for this initial interface check; subsequent synchronized motor/state integration may use a dedicated instantaneous-state plugin.

Install/launch:

```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select safe_redcho_swarm
source ~/ros2_ws/install/setup.bash
ros2 launch safe_redcho_swarm single_cf.launch.py
```

Stop other Gazebo/CrazySim launches for this isolated check. Select cf1 in the Gazebo entity tree and focus the camera if the 9 cm model is hard to see. The model should fall to the ground; hover is not expected before the motor plugin is added.

In a second sourced terminal:

```bash
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /cf1/odom --once
ros2 topic hz /cf1/odom
```

Stop the last command with Ctrl+C after several reports. The received wall-time frequency can differ from 100 Hz if the simulator's real-time factor differs from one or the machine is loaded.

For server-only mode use `ros2 launch safe_redcho_swarm single_cf.launch.py headless:=true`.

Local checks: Python syntax, XML structure, bridge YAML, physical parameters, and installation data-file paths were inspected. Gazebo/ROS are unavailable in the artifact-generation environment, so runtime acceptance must be confirmed on Ubuntu before adding the actuator plugin.
