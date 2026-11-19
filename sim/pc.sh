#!/usr/bin/env bash
# Host side for sim/run.sh: RViz with the simulator layout, plus joy_node if a gamepad is attached.
# Needs ros-humble-rviz2 (and ros-humble-joy for a gamepad) on this PC.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} ROS_LOCALHOST_ONLY=1
PIDS=()
if compgen -G '/dev/input/js*' >/dev/null; then
  ros2 run joy joy_node --ros-args -p device_id:="${JOY_DEVICE_ID:-0}" \
    -p autorepeat_rate:=20.0 -p deadzone:=0.05 &
  PIDS+=($!)
else
  echo 'No gamepad (/dev/input/js*): RViz only.' >&2
fi
trap 'kill ${PIDS[*]} 2>/dev/null' EXIT
rviz2 -d sim/rviz/sim.rviz
