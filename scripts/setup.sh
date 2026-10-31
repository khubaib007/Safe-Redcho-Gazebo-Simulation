#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-help}" in
  numerical)
    sudo apt update
    sudo apt install -y python3-numpy python3-scipy python3-matplotlib
    ;;
  ros)
    if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
      echo 'Install ROS 2 Jazzy first; see docs/installation.md.' >&2; exit 2
    fi
    sudo apt update
    sudo apt install -y build-essential cmake python3-colcon-common-extensions python3-rosdep       python3-numpy python3-scipy python3-matplotlib ros-jazzy-ros-gz ros-jazzy-rviz2       ros-jazzy-gz-sim-vendor ros-jazzy-gz-plugin-vendor       ros-jazzy-gz-transport-vendor ros-jazzy-gz-msgs-vendor \
      ros-jazzy-gz-gui-vendor ros-jazzy-gz-rendering-vendor qtbase5-dev qtdeclarative5-dev
    source /opt/ros/jazzy/setup.bash
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
    rosdep update
    rosdep install --from-paths "$root/src" --ignore-src --rosdistro jazzy -y
    ;;
  *) echo 'Usage: bash scripts/setup.sh {numerical|ros}'; exit 2 ;;
esac
