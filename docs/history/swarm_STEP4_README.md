> Historical delivery note. Use the root README and current docs/ guides for this release.

# Step 4: motor-thrust interface and diagnostic hover

This incremental ZIP adds the C++ `safe_redcho_gazebo` package and updates `safe_redcho_swarm` with `hover_check.py`, a new launch file/world, and its executable registration. It leaves baseline/controller modules and the Step 3 world unchanged.

## Install and run

Stop the Step 3 Gazebo launch with Ctrl+C. Extract the ZIP into ~/ros2_ws/src.

```bash
sudo apt install build-essential cmake ros-jazzy-gz-sim-vendor ros-jazzy-gz-plugin-vendor
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select safe_redcho_gazebo safe_redcho_swarm --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_ws/install/setup.bash
ros2 launch safe_redcho_swarm hover_check.launch.py
```

For server-only mode append `headless:=true`. The launch file adds the installed plugin library directory to Gazebo's plugin path. No manual global environment change is required.

## Expected behavior

The model starts at z=1 m. It may fall briefly, or reach the ground, while ROS discovery establishes communication. The diagnostic position controller should then recover and hold z=1 m. It logs height, vertical velocity, attitude error, and collective thrust once per simulated second. After 15 seconds from the first state, it reports a hover pass/fail using its last five seconds: max height error <0.02 m, max total speed <0.03 m/s, and max XY displacement <0.02 m. It continues hovering until the launch is stopped. At equilibrium total thrust should approach m*g =0.293319 N, or 0.07332975 N per motor.

CSV output: ~/ros2_ws/results/safe_redcho_hover/hover.csv. Stop the integrated launch to stop both the simulator and hover node; stopping just the hover node leaves an unpowered simulated vehicle after the command timeout.

## Interface

- `/cf1/motor_command`: std_msgs/msg/Float64MultiArray, data=[f1,f2,f3,f4,Fmax], all in newtons. Five values are required. Fmax is an atomic per-command actuator cap, limited to nominal 0.16 N. Negative motor requests are clamped to zero; oversized requests are clamped to Fmax. Non-finite/malformed messages are rejected.
- `/cf1/motor_applied`: same array type and ordering, reporting applied thrusts and cap. This diagnostic message has no timestamp; correlated stamped actuator feedback will be added for synchronized swarm execution.
- `/cf1/state`: nav_msgs/msg/Odometry directly from Gazebo physics, configured for 100 Hz. Pose is in world coordinates; BOTH linear and angular twist are in the body/child frame. The hover controller transforms linear velocity to world coordinates and uses angular velocity in body coordinates.
- `/cf1/odom`: retains the existing bridged Gazebo OdometryPublisher topic for comparison.
- `/clock`: retains the existing world-qualified bridge.

The ROS subscriber is spun on the simulation thread in PreUpdate. The entire five-value motor command is read atomically; no callback mutates physics from a worker thread. Forces and torques are applied every physics step. The plugin rotates body thrust/torque into world coordinates and adds quadratic drag -D*||v||*v. Gravity and rigid-body integration are handled only by Gazebo. No second motor/firmware controller is active.

Motor mixing matches the original Python allocation matrix exactly. The same geometric controller is used. The hover check adds a temporary position PD and an acceleration clamp solely for diagnostics; REDCHO, SVF, battery updates and HOCBF are not active in this check. Motor limits are nominal for this stage; mission voltage-dependent caps will use the same fifth message field later.

The plugin has a 0.25-second simulation-time stale-command timeout, after which thrust becomes zero. This is simulation-interface behavior, not a hardware landing controller. This first interface is asynchronous and does not yet provide the lockstep/acknowledgment protocol planned for the swarm controller.

## Checks

In another sourced terminal:

```bash
ros2 topic echo /cf1/state --once
ros2 topic echo /cf1/motor_applied --once
ros2 topic hz /cf1/state
```

Paste the HOVER CHECK report and the applied motor message. If the plugin fails to build/load, paste its first compiler/load error.

## Validation limits

The standalone C++ motor mixing was compiled and compared with the Python mixing; the hover diagnostic was exercised against the source SE(3) integrator at a 1 ms physics step and 100 Hz controller rate; Python/XML/configuration were checked. ROS/Gazebo development libraries are unavailable in the delivery environment, so the complete plugin must be compiled and run on the Ubuntu computer. A successful hover does not validate all attitude axes, the sampled safety assumptions, or the full swarm; those are subsequent steps.
