# Validation status: velocity-v0.1.0

Release preparation date: 31 October 2026.

| Tier | Evidence |
|---|---|
| Numerical/protocol suite | PASS; see `validation/numerical_tests.json` |
| 45-second numerical mission | PASS, 4,501 cycles; see `validation/numerical_mission_status.json` |
| Numerical safety/consensus metrics | `validation/numerical_performance_summary.json` |
| Numerical backend/parameters | `validation/numerical_metadata.json` |
| Native clean colcon build | PASS, all eight packages; see `validation/native_build.log` |
| Native 45-second Gazebo mission | PASS, 4,501 accepted batches and six distinct controller processes; see `validation/native_mission_status.json` |
| GitHub Actions | Workflow included; first remote result is separate from local evidence |
| Hardware | Not part of this simulation repository |

The numerical runner uses a midpoint SE(3) plant and simple floor-contact proxy.
Protocol checks use ROS doubles; neither constitutes native Gazebo execution.
The controller uses sampled, epsilon-smoothed REDCHO and numerical QP solving.
The reported tests establish their tested scenarios, not a general continuous-time
or hardware safety guarantee.

Native build/run evidence, when present, names the exact completed duration and
backend. The repository's scripts and docs can describe full native commands even
when a given release has only a native smoke test; the status must distinguish them.

## Native release result

The full 45-second headless Gazebo run passed all 6 evaluated checks. Minimum pair spacing was 0.914224 m, maximum QP violation was 5.386e-16 and Phase-I slack was 0.000e+00. Native and numerical reports remain separate. GUI rendering was not exercised by the headless native test.
