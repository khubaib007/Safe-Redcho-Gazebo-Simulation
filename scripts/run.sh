#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"
cd "$REDCHO_ROOT"
mode="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
  numerical) exec /usr/bin/python3 scripts/numerical.py "$@" ;;
  baseline) exec /usr/bin/python3 -m safe_redcho_swarm.nodes.baseline --no-show --no-animation "$@" ;;
  world|agents|gazebo)
    if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f install/local_setup.bash ]]; then
      echo 'ROS/build missing; see docs/installation.md.' >&2; exit 2
    fi
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER
    export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
    source /opt/ros/jazzy/setup.bash
    source install/local_setup.bash
    source scripts/environment.sh
    export ROS_LOG_DIR="${ROS_LOG_DIR:-$REDCHO_ROOT/log/ros}"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export GZ_PARTITION="${GZ_PARTITION:-safe_redcho_velocity}"
    case "$mode" in
      world) exec ros2 launch safe_redcho_scene arena.launch.py "$@" ;;
      agents) exec ros2 launch safe_redcho_distributed control_agents.launch.py "$@" ;;
      gazebo) exec ros2 run safe_redcho_distributed gazebo_mission "$@" ;;
    esac ;;
  *) echo 'Usage: bash scripts/run.sh {numerical|baseline|world|agents|gazebo} [arguments]'; exit 2 ;;
esac
