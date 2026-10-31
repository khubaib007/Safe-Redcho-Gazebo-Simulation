#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
/usr/bin/python3 "$REDCHO_ROOT/scripts/verify.py"
if [[ "${1:-}" == --ros ]]; then
  source /opt/ros/jazzy/setup.bash
  if [[ ! -f "$REDCHO_ROOT/install/local_setup.bash" ]]; then echo 'Build first with bash scripts/build.sh' >&2; exit 2; fi
  source "$REDCHO_ROOT/install/local_setup.bash"
  /usr/bin/python3 - <<'PY'
from safe_redcho_interfaces.srv import StepSwarm
from safe_redcho_distributed_interfaces.msg import ControlFrame, ControlResult, SafetyBounds
from ament_index_python.packages import get_package_prefix
from pathlib import Path
import os
root=Path(os.environ['REDCHO_ROOT']).resolve()
for package,library in [('safe_redcho_lockstep','libsafe_redcho_lockstep_system.so'),('safe_redcho_visualization','libSafeRedchoPropellers.so')]:
    prefix=Path(get_package_prefix(package)).resolve()
    if not prefix.is_relative_to(root/'install'):
        raise SystemExit('Wrong overlay for '+package+': '+str(prefix))
    target=prefix/'lib'/library
    if not target.is_file(): raise SystemExit('Missing native library: '+str(target))
print('PASS: generated ROS interfaces and native libraries resolve to this checkout')
PY
fi
