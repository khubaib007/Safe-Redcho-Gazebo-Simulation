# Single position REDCHO Gazebo baseline

`safe_redcho_single_sim` is a separate ROS 2 Jazzy/Gazebo Harmonic package. Six independent agents each run **one order-2 position REDCHO bank**, a local battery model and SVF, the existing HOCBF-QP, geometric controller and motor allocation. Position-bank derivative outputs supply velocity and acceleration. Existing packages remain untouched.

The current ground-start formation scenario is retained: six Crazyflies, distinct height proposals averaging 0.8 m, hover gate, battery-aware forward flight, two obstacles, cf2 charge/resistance fault at 17 s and final formation recovery. World geometry, dynamics, gains, safety margins and fault laws are preserved. Trajectories may differ because the controller reference architecture changes. The simulation ends paused in flight, as the original experiment does.

See [ANALYSIS.md](ANALYSIS.md) for the existing architecture and scope, [MATHEMATICAL_FORMULATION.md](MATHEMATICAL_FORMULATION.md) for equations, and [RUN_COMMANDS.md](RUN_COMMANDS.md) for build, tests, headless and GUI execution. For this portable release use the [complete repository guide](../../docs/single-position/START_HERE.md) and [publication validation](../../docs/single-position/VALIDATION.md). The original workspace validation record is retained as historical evidence.

The [validation report](VALIDATION.md) records the full native 45 s mission, numerical nominal/disturbance cases, 37 passing tests and source-preservation audit.

Control topics use `/safe_redcho_single_sim`. The shared native backend retains `/safe_redcho/step` and `/cfN/state`; use one fresh world per ROS domain/Gazebo partition. The recommended new domain is 47. The local observer never receives a centrally computed average. Only diagnostics compute averages as error oracles.

Outputs include `mission_status.json`, `performance_summary.json`, `metadata.json`, `simulation_data.npz`, complete per-agent CSV, global CSV and eight saved scientific figures. Failed runs retain partial data. Velocity-bank and Levant REDCHO arrays are absent, rather than populated with invented values. Gazebo and RViz provide live views; saved plotting uses a noninteractive backend.

For a fair paper comparison, use the same source snapshot, geometry, physical parameters, fault schedule, time interval and safety checks in both experiments. This is a simulation baseline; it is not hardware flight validation, firmware execution, or a proof of recursive feasibility/universal obstacle avoidance.
