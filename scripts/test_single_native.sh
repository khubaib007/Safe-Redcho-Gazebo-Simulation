#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-smoke}" in
  smoke) duration=2 ;;
  full) duration=45 ;;
  *) echo 'Usage: bash scripts/test_single_native.sh {smoke|full}'; exit 2 ;;
esac
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH VIRTUAL_ENV
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_STATIC_PEERS ROS_DISCOVERY_SERVER
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/jazzy/setup.bash
source "$root/install/local_setup.bash"
source "$root/scripts/environment.sh"
cd "$root"
export ROS_DOMAIN_ID="${REDCHO_TEST_DOMAIN_ID:-196}"
export ROS_LOCALHOST_ONLY=1
export GZ_PARTITION="single_position_validation_publish_$(date -u +%Y%m%dT%H%M%SZ)_$$"
output="$root/results/native_single_$(date -u +%Y%m%dT%H%M%SZ)_$$"
export ROS_LOG_DIR="$output/ros_logs"
exec /usr/bin/python3 -s src/safe_redcho_single_sim/test/run_gazebo_check.py \
  --duration "$duration" --plots --output "$output"
