# Single position REDCHO: complete simulation guide

This release is a complete ROS 2 Jazzy / Gazebo Harmonic workspace with the
separate package `safe_redcho_single_sim`. Each of six controller processes owns
one order-2 position REDCHO bank. Its derivative outputs provide velocity and
acceleration; there is no independent velocity REDCHO bank. HOCBF-QPs, battery
fault model, geometric controller and motor allocation remain in the loop.

## 1. Download and test without ROS

Ubuntu 24.04, Python 3.12:

```bash
git clone --branch single-position-v0.1.0 --depth 1 \
  https://github.com/khubaib007/Safe-Redcho-Gazebo-Simulation.git safe-redcho-single-sim
cd safe-redcho-single-sim
bash scripts/setup.sh numerical
bash scripts/verify.sh
bash scripts/test.sh full
bash scripts/run.sh numerical --duration 45 --plots --output results/first_single_numerical
```

The full test runner includes the package's unit/protocol/cleanup tests and a
45-second numerical mission. Numerical simulation uses a SE(3) plant and floor
proxy; it does not run Gazebo. Use a fresh output directory for every repeat.

For the bounded acceleration-disturbance experiment:

```bash
bash scripts/run.sh numerical --duration 45 --disturbance \
  --output results/single_numerical_disturbance
```

Numerical figures are saved by default. `--no-plots` skips saved figures;
`--no-csv` disables CSV export. NPZ and status/metadata remain available.

## 2. Install ROS and build the complete checkout

Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
then start a fresh terminal without a hardware workspace or virtual environment:

```bash
cd /path/to/safe-redcho-single-sim
bash scripts/setup.sh ros
bash scripts/build.sh --executor sequential
bash scripts/verify.sh --ros
bash scripts/run.sh doctor
/usr/bin/python3 scripts/audit_single.py
```

The build includes all nine packages, generated interfaces, the lockstep motor
physics plugin and visualization plugin. The doctor verifies the frozen shared
equations, world geometry/mass/inertia, interfaces and native library. It does
not run the mission. The publication source audit verifies the copied package,
worlds and shared equations. The original 139-file workstation audit remains
historical evidence; it is not the audit for this portable checkout.

## 3. Full native Gazebo run, no desktop windows

One command starts a new paused physical world, six separate controllers, clock
bridge, view publisher and mission coordinator:

```bash
bash scripts/run.sh simulation \
  headless:=true rviz:=false plots:=false csv:=true duration:=45.0 \
  output:="$PWD/results/single_native_$(date -u +%Y%m%dT%H%M%SZ)"
```

`plots:=false` disables saved scientific figures; it does not disable CSV/NPZ
telemetry. There are no interactive plot windows. Add
`shutdown_on_finish:=true` for automatic scoped shutdown after the coordinator
finishes. Always inspect `mission_status.json`: process shutdown alone does not
mean PASS. With the default setting, Gazebo remains paused after the mission.

The wrapper defaults to ROS domain **47** and partition
`safe_redcho_single_sim`. It clears incompatible hardware Fast DDS settings.
Use only one fresh world and coordinator per domain/partition. Never press Play
or issue manual stepping requests; the coordinator owns physics stepping.

## 4. Gazebo GUI, RViz, telemetry and saved figures

```bash
bash scripts/run.sh simulation \
  headless:=false rviz:=true view:=closeup plots:=true csv:=true duration:=45.0 \
  output:="$PWD/results/single_gui_$(date -u +%Y%m%dT%H%M%SZ)"
```

Gazebo/RViz are live views. `plots:=true` writes eight PNG scientific figures
after the run using a noninteractive plotting backend. The inherited ground
start, height proposals, hover gate, battery-aware forward mission, two
obstacles, cf2 fault at 17 seconds and final recovery are retained. The mission
ends paused in flight; it is not a simulated landing experiment.

## 5. Automated isolated native verification

```bash
bash scripts/test_native.sh smoke
bash scripts/test_native.sh full
```

Smoke covers two seconds of transport/ground startup. Full covers the complete
45-second mission and checks all 4501 accepted batches, six distinct agent
processes, 27006 per-agent CSV rows, NPZ finiteness/schema, all mission metrics,
eight saved figures and scoped Gazebo process cleanup. It uses a unique
`single_position_validation_...` partition and isolated ROS domain 196 by
default. Override only the test domain with `REDCHO_TEST_DOMAIN_ID` if needed;
do not select a domain used by another experiment or hardware.

The test launcher cleans up only its own Gazebo/agent processes. Evidence is
saved under `results/native_single_*/`. No aircraft hardware is accessed.

## 6. Separate terminals for debugging

In each terminal, use this checkout and the same domain/partition. Start a fresh
world and fresh agents for every mission:

```bash
# Terminal 1
bash scripts/run.sh world headless:=false rviz:=true
```

```bash
# Terminal 2
bash scripts/run.sh agents
```

```bash
# Terminal 3
bash scripts/run.sh gazebo --duration 45 --no-plots \
  --output "$PWD/results/single_debug_$(date -u +%Y%m%dT%H%M%SZ)"
```

After completion stop agents and world with Ctrl+C. The Gazebo supervisor owns
its child process group and records `gazebo_shutdown.json`; unrelated runs are
outside that group. Do not reuse stepped world, battery, agent or observer
states for another run. The combined `simulation` launcher is the recommended
ordinary entry point.

## 7. Focused tests and outputs

```bash
source scripts/environment.sh
/usr/bin/python3 -s -m pytest -q src/safe_redcho_single_sim/test
/usr/bin/python3 scripts/audit_single.py
```

Each complete run writes `mission_status.json`, `performance_summary.json`,
`metadata.json`, `simulation_data.npz`, `agent_telemetry.csv`,
`global_telemetry.csv` and `telemetry_schema.json`. Figures cover trajectories,
local references, single-bank REDCHO errors, safety, batteries, formation,
motor/plant behavior and 3D paths. Partial records are retained on failure.
Velocity-bank arrays are absent rather than filled with placeholder values.

## Troubleshooting

| Symptom | Action |
|---|---|
| Missing plugin/interface | Build this entire checkout in a clean terminal, then run `verify.sh --ros` and `run.sh doctor`. Do not source the hardware overlay. |
| Frozen-core/world hash mismatch | Restore the release's shared core and worlds, then rebuild. This baseline intentionally uses frozen equations and geometry. |
| Step timeout, duplicate coordinator or nonzero first sequence | Stop that launch, use a fresh domain/partition and restart both world and agents. Never reuse a stepped world or press Play. |
| No GUI/display | Run headless, or use a graphical desktop with Gazebo/RViz support. Saved figures do not require a display. |
| No PNG figures | Set `plots:=true`; this controls saved figures, not live Gazebo/RViz or CSV. |
| Output already exists | Select a new timestamped directory. The runner refuses to overwrite a completed dataset. |
| Slow wall-clock simulation | Lockstep is allowed to run slower than real time. Judge the simulated 45-second interval and completed batches; do not force manual stepping. |
| Failed performance check | Inspect status, summary and CSV/NPZ evidence. Keep failure records; do not treat process exit alone as mission success. |

See the [architecture](../../src/safe_redcho_single_sim/ANALYSIS.md),
[full mathematical formulation](../../src/safe_redcho_single_sim/MATHEMATICAL_FORMULATION.md)
and [publication validation](VALIDATION.md). This edition validates simulation,
not hardware flight or firmware execution.
