# Validation of the single-position REDCHO baseline

The separate package was built and tested with ROS 2 Jazzy and Gazebo Harmonic 8.15.0. No existing simulation source package or hardware package was edited.

- **37 tests passed**: independent vectorized REDCHO oracle, average conservation, nontrivial height consensus, single-bank derivative control, missing/malformed data, complete-batch/native-response barriers, CSV/NPZ schema and owned-process cleanup.
- **Full native Gazebo mission passed**: six distinct agent processes, 4501 accepted motor batches, 27,006 agent CSV rows and eight saved scientific figures.
- **Numerical nominal and bounded-disturbance missions passed**, including the cf2 fault and both obstacles.
- **139 original source/asset hashes preserved**. Shared controller/physical core hashes and copied world geometry are checked by `doctor`.
- **Final native cleanup passed**: Gazebo CLI and native child stopped cleanly on SIGINT, with no remaining owned process group. Initial testing exposed a wrapper-child orphan; the new package's process supervisor fixes it without altering the shared native plugin.

Native-run metrics: minimum pair distance **0.912876 m** against 0.7 m physical requirement; minimum obstacle surface clearances **1.281657 m** and **1.269506 m** against 0.5 m; maximum free-flight acceleration residual **0.213968 m/s²** against 0.4 m/s²; no Phase-I invocations and maximum hard-QP violation about **4.29e-15**. Over the final five seconds, maximum shape error was 0.012714 m, height spread 0.003767 m and velocity spread 0.006757 m/s.

The repeated native run with the corrected process supervisor reproduced the recorded physical states, motor commands and position REDCHO outputs exactly in this environment. This is an observed repeatability result, not a cross-platform bitwise guarantee.

Detailed evidence is in [the workspace validation report](../../results/single_position_validation_20261106/VALIDATION.md). Native Gazebo was executed headless. Saved plots were rendered and inspected; desktop Gazebo/RViz window rendering was not exercised. The nominal/native results describe this fixed scenario and model, not arbitrary geometry, firmware/SITL, hardware flight or a proof of continuous-time/recursive safety.
