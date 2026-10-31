# Troubleshooting

| Symptom | Diagnose | Resolution |
|---|---|---|
| `numpy.long` or SciPy import failure | Check versions printed by `bash scripts/verify.sh` | Use the wrappers and system Python; they set `PYTHONNOUSERSITE=1`. Exit Conda/old virtual environments. |
| `ros2`, `colcon` or Jazzy missing | Check `/opt/ros/jazzy/setup.bash` | Follow installation.md, then run setup/build. |
| `gz-*` or Qt5 CMake package missing | Read the first missing dependency in the build log | Run `bash scripts/setup.sh ros`; ensure ROS Jazzy base is installed. |
| Generated message import fails | Run `bash scripts/verify.sh --ros` | Build all eight packages in this checkout. Do not reuse another edition's install directory. |
| Package resolves to an old workspace | Inspect prefix reported by ROS verification | Open a fresh terminal; use this checkout's wrappers and rebuild here. |
| World/controller geometry mismatch | Run `bash scripts/verify.sh` | Regenerate with `src/configure_scenario.py`, retest, rebuild and restart world/agents. |
| Coordinator waits for readiness | Read ready-agent IDs in the timeout error | Start all six agents on the same ROS domain. Stop extra ring/observer launches and duplicate coordinators. |
| Gazebo step service absent | Confirm Terminal 1 is still running and native build passed | Use the same `ROS_DOMAIN_ID` and `GZ_PARTITION`; start the included scene world. |
| Sequence/epoch/bootstrap rejection | Check if world or agents were reused | Stop both, start a new paused world and six new agents. |
| Native physics step count/time mismatch | Check manual Play or other step clients | Keep the world paused; allow only the mission coordinator to advance it. |
| Hard QP infeasible / nonfinite output | Preserve mission status and partial histories | Review geometry, authority and state data. Do not disable strict checks or accept Phase-I slack as safe control. |
| Plant residual exceeds 0.40 | Inspect backend, initialization and prior commands | Verify matched physics parameters and fresh world. A different plant requires a justified model/bound review. |
| Full formation check absent | Inspect requested duration | Run 45 seconds; a 2-second smoke run cannot evaluate final recovery. |
| Numerical output directory rejected | Inspect its existing files | Pick a new output name; the wrapper protects previous numerical results. |
| No GUI / OpenGL error | Try the headless README sequence | Use `headless:=true rviz:=false` for nonvisual validation; configure desktop graphics separately. |
| Figures missing | Inspect plotting flags and final status | Numerical mode needs `--plots`; native mode needs omission of `--no-plots`. |

For a useful issue report include edition/tag, OS, numerical versions, command,
the first error, mission status, performance summary and metadata. Include
partial histories if needed; avoid uploading unrelated local editor/build files.

After intentional Ctrl+C shutdown, ROS launch/Gazebo may print KeyboardInterrupt or exit code -2. Judge mission completion from the saved final report before teardown; unexpected exits during a mission still require investigation.
