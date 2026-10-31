> Historical delivery note. Use the root README and current docs/ guides for this release.

# Safe REDCHO Swarm — Step 2: numerical baseline

This ROS Python package places the previously delivered numerical controller in `safe_redcho_swarm/core`, all figures in `safe_redcho_swarm/visualization/figures.py`, and the numerical experiment runner in `safe_redcho_swarm/nodes/baseline.py`.

The baseline executable is packaged with ROS but deliberately performs no Gazebo integration and spins no ROS control node yet. It uses the original Python rigid-body integrator to establish the expected numerical results. This stage verifies packaging and numerical dependencies before changing the physical backend.

## Install into the package created in Step 1

The ZIP has a single top-level `safe_redcho_swarm/` folder. Extract it into `~/ros2_ws/src/`, merging it with the generated package. It replaces the generated `setup.py`, `setup.cfg`, `package.xml`, and placeholder `main.py`, and adds the controller modules. Existing files not present in the ZIP are left in place.

Ubuntu dependencies:

```bash
sudo apt install python3-numpy python3-scipy python3-matplotlib
```

Use Ubuntu's `/usr/bin/python3` with the ROS Jazzy environment. Avoid activating the CrazySim virtual environment for this step. The controller uses numerical APIs supported by Ubuntu's SciPy package; package-version differences may slightly alter numerical results.

Build and run:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select safe_redcho_swarm
source ~/ros2_ws/install/setup.bash
ros2 run safe_redcho_swarm baseline --duration 1 --no-plots --output "$HOME/ros2_ws/results/safe_redcho_smoke"
ros2 run safe_redcho_swarm baseline --no-show --output "$HOME/ros2_ws/results/safe_redcho_baseline"
```

The first command is a one-second smoke run; it does not exercise the fault. The second runs the full 40-second numerical baseline, including the fault, and saves all seven figure PNGs without opening windows. For interactive plots and playback, omit `--no-show`.

The full run should report all five checks as true, 24,006 agent-steps, and zero Phase-I invocations. In the validated reference environment: minimum spacing 1.01247 m; maximum acceleration residual 0.23801 m/s²; SVF certificate 17.99273 m/s⁴; selected REDCHO L=27.43. Small differences across dependency versions are expected. A failed check or interrupted run must be reviewed before Gazebo integration.

## Files

- `core/config.py`: parameter classes.
- `core/battery.py`: voltage, speed governor, and actuator authority.
- `core/reference_filters.py`: SVF, its certificate, and Levant ablation.
- `core/redcho.py`: REDCHO matrices, propagation, perturbation kernel.
- `core/cbf_qp.py`: reciprocal HOCBF-QP and Phase-I.
- `core/quadrotor.py`: geometric controller, motor mixing, geometry, and baseline plant integrator.
- `visualization/figures.py`: all plots and playback.
- `nodes/baseline.py`: full baseline loop, CLI, and numerical report.
- `main.py`: package status entry point.

This preserves the attached MATLAB-code scenario, which differs from the paper's obstacle locations/radii and one initial charge. SciPy replaces MATLAB quadprog, as in the previously delivered Python version. No Gazebo equivalence or direct MATLAB execution comparison is claimed.
