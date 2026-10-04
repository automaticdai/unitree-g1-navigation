#!/usr/bin/env bash
set -euo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo_root"
if [[ ${ROS_DISTRO:-} != humble ]]; then
  echo 'Source /opt/ros/humble/setup.bash first.' >&2
  exit 1
fi
mkdir -p vendor/ros vendor/hardware
vcs import vendor/ros < dependencies.repos
vcs import vendor/hardware < hardware.repos
git -C vendor/ros/fast_lio submodule update --init --recursive
# Select the driver's ROS 2 manifest without invoking its destructive build.sh.
cp vendor/ros/livox_ros_driver2/package_ROS2.xml vendor/ros/livox_ros_driver2/package.xml
mkdir -p vendor/ros/livox_ros_driver2/launch
cp -r vendor/ros/livox_ros_driver2/launch_ROS2/. vendor/ros/livox_ros_driver2/launch/
echo 'Pinned sources prepared. See README for SDK installation and colcon build.'
