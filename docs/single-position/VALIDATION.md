# Single-bank simulation provenance and validation

Release: `single-position-v0.1.0`, package `safe_redcho_single_sim`, copied from
the latest `safe_redcho_distributed_ws` implementation on 6 November 2026.

All 28 package Python source files are copied byte for byte. Controller and
observer equations, scenario parameters, world geometry, dynamics, battery
faults and safety settings are unchanged. Publication adds portable shell entry
points, source auditing, CI coverage and documentation. The old velocity and
dual-bank editions remain unchanged at their existing tags.

The shared core modules and world hashes are verified against the single-bank
package's frozen original snapshot by `doctor` and `scripts/audit_single.py`.
The full original 139-file workstation preservation audit is retained as
historical evidence; the earlier GitHub editions intentionally adapted some
package metadata/docs and omitted historical development files. Publication
does not claim those 139 workstation files all match the portable repository.

`validation/single_source_snapshot.json` records original package hashes;
`single_publication_validation.json` records the fresh publication checks.
`single_original_*` files describe the earlier workstation native/numerical
validation, including the 4501-cycle actual Gazebo mission and 37 tests.

Fresh publication validation covers:

- Building all nine packages from this checkout, including native plugins.
- Actual generated ROS interface and native-library lookup in this install.
- Frozen shared core/world/physical-parameter checks and the local doctor.
- Existing protocol checks, all 37 single-bank tests, and a complete nominal
  45-second numerical mission.
- An isolated full native Gazebo mission with six distinct controller processes,
  accepted motor batches, telemetry schemas, finite arrays, all performance
  checks, eight saved figures and scoped process cleanup.

The native validation archive attached to the release contains fresh run
evidence and outputs. Scientific figures are also available in `images/`.
These tests validate this simulation implementation and its stated scenario;
they do not validate physical aircraft, firmware, universal obstacle avoidance
or recursive QP feasibility for arbitrary scenarios. Desktop Gazebo/RViz GUI
behavior has not been requalified by the headless validation.

For an architecture-only paper comparison, match gains, world/plant parameters,
formation offsets, fault schedule, initial state, simulation interval and
performance checks in both editions. Do not attribute parameter or scenario
changes to the bank count.
