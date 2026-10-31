# Testing

```bash
bash scripts/test.sh quick
bash scripts/test.sh full
```

Quick tests parse Python/XML/Bash, verify current scenario/world consistency,
exercise the ring protocol, inject invalid QP results, check coordinator barriers,
validate native response decoding, and verify the physics/mesh fingerprint.
The formation edition also checks formation-reference influence and control-agent
message ordering. Protocol tests use explicit ROS doubles in separate processes.

The full suite adds a complete 45-second numerical mission and generates plots.
It verifies local command feedback through a numerical SE(3) plant, battery
fault response, safety checks and the edition's applicable consensus checks.

CI in `.github/workflows/numerical.yml` runs the full suite on Ubuntu 24.04 and
uploads its evidence. It does not claim to run native Gazebo. First GitHub Actions
execution may still expose runner-specific dependency differences; inspect its
report rather than assuming a local PASS means CI has passed.

Native testing is a separate tier: a clean colcon build, interface verification,
and the three-terminal Gazebo smoke/full runs. See validation-status.md for which
tiers were actually completed during preparation.

The original reference-file-specific tests depended on obsolete fixture/API
assumptions. Their supported replacement is the standalone numerical full runner;
see provenance.md. Source snapshots/hashes retain the origin of those changes.

## Automated native tests

After building, run `bash scripts/test_native.sh smoke` or `bash scripts/test_native.sh full`. These start headless world, agents and coordinator, preserve all logs under a new results/native_<UTC>/ directory, and stop only their own process groups. The default test ROS domain is 162 and the Gazebo partition is unique per test. Override the domain with REDCHO_TEST_DOMAIN_ID when required.
