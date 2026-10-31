# Running Safe REDCHO Velocity

All commands run from the checkout root. Shell wrappers resolve their own paths;
no fixed `/home/khubaib` directory is required.

## Numerical smoke and full mission

```bash
bash scripts/run.sh numerical --duration 2 --output results/numerical_smoke
bash scripts/run.sh numerical --duration 45 --plots --output results/numerical_full
```

The 2-second run checks startup execution only. The 45-second run exercises
takeoff, hover, obstacles and battery fault. Formation recovery is evaluated
only in the formation edition and only with sufficient duration (45 s by default).
Use a new output directory for every numerical run.

## Native Gazebo

Follow the three-terminal sequence in the README. The wrappers use ROS domain
42 by default. To select another domain, export the same `ROS_DOMAIN_ID` in all
three terminals. They also set a variant-specific Gazebo transport partition;
use the same `GZ_PARTITION` everywhere if overriding it.

For an initial native smoke test:

```bash
bash scripts/run.sh gazebo --duration 2 --no-plots --output results/gazebo_smoke
```

For full validation, restart the world and agents, then run 45 seconds. Wait for
the final `mission_status.json` and inspect its status and checks. The world
remains paused. Stop coordinator/agents/world with Ctrl+C; restart both the
world and controllers before any repeat. Do not start two coordinators on the
same domain/partition, manually unpause the world, or reuse old agent processes.

## Outputs

| File | Meaning |
|---|---|
| `mission_status.json` | Backend, outcome, cycle counts and failure information |
| `performance_summary.json` | Evaluated numerical/safety/consensus checks |
| `metadata.json` | Scenario, parameters, timing and backend provenance |
| `simulation_data.npz` | Complete successful state/controller histories |
| `partial_simulation_data.npz` | Available histories after failure |
| `figure_*.png` | Trajectory, safety, observer, battery and optional formation plots |

`--plots` enables numerical plots. For native plots omit `--no-plots`.
Logical mission time and wall execution time differ: paused stepping can run
slower than real time. Duration 45 s produces 4,501 control samples; the native
backend also applies the final sample's motor batch, so logged state horizon and
total plant advance differ by one 10 ms step. Metadata records both.

## Original numerical baseline comparator

```bash
bash scripts/run.sh baseline --duration 40 --output results/original_baseline
```

This is the older single-process numerical reference, not a six-process ROS
mission. It starts airborne and uses a different scenario. Its results must not
be presented as native Gazebo or formation-edition validation.
