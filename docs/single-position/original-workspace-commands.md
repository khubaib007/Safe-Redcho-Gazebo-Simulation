Historical workstation record. Use [START_HERE.md](START_HERE.md) for this portable release; original checkpoint/results directories below are not bundled.

# Build, test and run the separate single-bank simulation

Use `/usr/bin/python3` and ROS Jazzy. Start a fresh terminal without the hardware virtual environment. Existing packages and the original install are retained.

## Build the separate overlay

```bash
cd ~/safe_redcho_distributed_ws
set +u
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
/usr/bin/colcon --log-base log_single_redcho build --base-paths src \
  --build-base build_single_redcho --install-base install_single_redcho \
  --packages-select safe_redcho_single_sim
source install_single_redcho/local_setup.bash
```

If a shared dependency is missing, build that existing dependency before this overlay; do not mix the hardware workspace's install into this terminal. Required shared packages include `safe_redcho_swarm`, `safe_redcho_interfaces`, `safe_redcho_distributed_interfaces`, `safe_redcho_lockstep`, `safe_redcho_scene` and `safe_redcho_visualization`, plus `ros_gz_sim`/`ros_gz_bridge`.

## Read-only checks and unit tests

```bash
ros2 run safe_redcho_single_sim doctor
PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /usr/bin/python3 -s -m pytest -q src/safe_redcho_single_sim/test
```

`doctor` checks installed core hashes, generated interfaces, world/controller geometry and mass/inertia, native plugin availability, and the single-bank reference certificate. It does not step Gazebo or verify the live graph.

To repeat the source-preservation audit:

```bash
/usr/bin/python3 -s src/safe_redcho_single_sim/test/audit_baseline.py \
  --workspace "$PWD" --output "$PWD/results/single_position_audit"
```

## Offline numerical mission, with the actual local HOCBF-QPs

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  ros2 run safe_redcho_single_sim offline --duration 45 \
  --output "$PWD/results/single_position_offline_$(date -u +%Y%m%dT%H%M%SZ)"
```

This generates CSV/NPZ and figures with no plot windows. Add `--no-plots` to skip figure generation; CSV remains enabled. Add `--disturbance` to exercise the inherited bounded cf3/cf5 acceleration disturbance between 28 and 30 s. This is a numerical SE(3) plant with a floor proxy, **not** Gazebo.

## Native Gazebo mission, one command, all live plots off

In the overlay terminal:

```bash
export ROS_DOMAIN_ID=47
export GZ_PARTITION=safe_redcho_single_sim
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
ros2 launch safe_redcho_single_sim simulation.launch.py \
  headless:=true rviz:=false plots:=false csv:=true duration:=45.0 \
  output:="$PWD/results/single_position_gazebo_$(date -u +%Y%m%dT%H%M%SZ)"
```

The launch starts a fresh paused physical world, clock bridge, read-only view, six independent single-bank controllers and coordinator. `plots:=false` skips saved figures; it does not disable CSV or NPZ telemetry. The world remains paused after completion for inspection. Stop this launch before rerunning. For automated runs add `shutdown_on_finish:=true` to stop only these launched processes when the mission executable finishes. Always inspect `mission_status.json`: launch shutdown alone is not a PASS criterion.

The new launcher owns the Gazebo CLI and native child in a separate process group, with bounded signal escalation and descendant reaping. `gazebo_shutdown.json` records that cleanup when the launch stops. This avoids leaving a backend child behind if the Gazebo CLI wrapper exits first; unrelated simulation processes are outside this group.

For a short transport/ground smoke test set `duration:=2.0`. It does not cover takeoff, fault, obstacle bypass or final formation checks. A full 45 s run is required for those checks. Lockstep simulation may run slower than real time.

For an automated isolated native test with result/CSV/NPZ/cleanup assertions, use a separate sourced test terminal:

```bash
export ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1
export GZ_PARTITION="single_position_validation_$(date -u +%Y%m%dT%H%M%SZ)"
/usr/bin/python3 -s src/safe_redcho_single_sim/test/run_gazebo_check.py \
  --duration 45 --plots \
  --output "$PWD/results/single_position_native_test_$(date -u +%Y%m%dT%H%M%SZ)"
```

Close that test terminal afterwards; use the domain-47 experiment environment for ordinary runs.

## GUI Gazebo, RViz, recorded telemetry and saved figures

```bash
ros2 launch safe_redcho_single_sim simulation.launch.py \
  headless:=false rviz:=true view:=closeup plots:=true csv:=true duration:=45.0 \
  output:="$PWD/results/single_position_gui_$(date -u +%Y%m%dT%H%M%SZ)"
```

Gazebo/RViz are live views. `plots:=true` saves scientific PNGs after the run using a noninteractive backend. Keep Gazebo paused; the coordinator owns physics stepping. Do not press Play, issue manual step requests or run another coordinator against this world.

## Separate terminals, for debugging

In each terminal source Jazzy, `install/local_setup.bash`, and `install_single_redcho/local_setup.bash`, and export the same `ROS_DOMAIN_ID=47` and `GZ_PARTITION=safe_redcho_single_sim`.

Terminal 1:

```bash
ros2 launch safe_redcho_single_sim world.launch.py headless:=false rviz:=true
```

Terminal 2:

```bash
ros2 launch safe_redcho_single_sim agents.launch.py
```

Terminal 3:

```bash
ros2 run safe_redcho_single_sim gazebo_mission --duration 45 --no-plots \
  --output "$PWD/results/single_position_manual_$(date -u +%Y%m%dT%H%M%SZ)"
```

Restart both world and agents for every run. The world sequence, local battery and REDCHO states intentionally cannot be reused from cycle 1. Both safety bounds and six agent results must match the current cycle. Extra subscriptions to the private `position_neighbor` topics affect the exact two-peer startup check; inspect the result topics or saved telemetry instead.

## Results and troubleshooting

PASS requires accepted motor batches and per-agent result counts to match expected cycles, plus every evaluated performance check. The 45 s inclusive record uses 4501 batches and 45.01 s of controlled physics after the 1 ms bootstrap. Shared safety context contains all six physical states; estimator communication contains only two neighbor estimates.

- **Missing package/interface or plugin:** re-source the simulation underlay/overlay, build the named dependency, and rerun `doctor`. Keep `PYTHONNOUSERSITE=1`; avoid the hardware venv.
- **Waiting for agents/backend:** confirm all terminals share ROS domain and Gazebo partition. Ensure six fresh processes, a paused fresh world, and one native backend. Old dual-bank controllers cannot satisfy the new private topic/implementation handshake.
- **Bootstrap/sequence error:** stop and restart this experiment's world and agents. Do not bypass response timestamp, wrench-count or applied-force checks.
- **Hard QP/residual/performance failure:** inspect `mission_status.json`, `performance_summary.json` and partial NPZ/CSV. Preserve the failed run; do not increase slack or modify safety margins merely to obtain PASS.
- **GUI unavailable:** use the headless command. GUI rendering is separate from controller/physics validation.
- **Output directory:** use a new timestamped directory for each run to avoid confusing older PASS files with a current run.

The new package does not modify Crazyflie hardware configuration or send hardware radio commands. The native simulation model is a surrogate with a software battery, not firmware/SITL.
