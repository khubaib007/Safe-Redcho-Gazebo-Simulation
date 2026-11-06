# Single-bank simulation commands

Use the [complete portable repository guide](../../docs/single-position/START_HERE.md)
for installation, build, unit/protocol checks, source audit, offline HOCBF-QP
missions, native headless tests, GUI/RViz, CSV/NPZ telemetry, figures and shutdown.

```bash
bash scripts/run.sh numerical --duration 45 --plots --output results/first_single_run
bash scripts/run.sh simulation headless:=false rviz:=true plots:=true csv:=true duration:=45.0
bash scripts/test_native.sh full
```

[Original workstation commands](../../docs/single-position/original-workspace-commands.md)
assume the old dedicated overlays. This checkout builds the complete workspace
into `install/`; do not use that historical overlay sequence here.
