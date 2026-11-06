#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then echo 'ROS Jazzy is missing; see docs/installation.md.' >&2; exit 2; fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
cd "$root"
exec /usr/bin/colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
