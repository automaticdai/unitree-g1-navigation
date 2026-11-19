#!/usr/bin/env bash
# Run the ROS Humble test tier in a container, mirroring .github/workflows/test.yml.
# Usage: bash scripts/ros_test.sh [pytest args...]   (SMOKE=1 also runs smoke scripts)
set -u
cd "$(dirname "$0")/.."

docker run --rm \
  -v "$PWD":/ws -w /ws \
  -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" -e SMOKE="${SMOKE:-0}" \
  ros:humble-ros-base bash -c '
    # Container runs as root; hand root-created files back to the host user on exit.
    trap "chown -R \"$HOST_UID:$HOST_GID\" build install log .pytest_cache scripts tests src 2>/dev/null; find . -xdev -user 0 -not -path ./.git/\* -exec chown \"$HOST_UID:$HOST_GID\" {} + 2>/dev/null" EXIT
    set -e
    apt-get update -qq
    apt-get install -y -qq python3-scipy python3-pytest ros-humble-message-filters ros-humble-sensor-msgs-py ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-collision-monitor ros-humble-joy
    source /opt/ros/humble/setup.bash
    colcon build --base-paths src --symlink-install
    source install/setup.bash
    set +e
    python3 -m pytest -q "$@"
    rc=$?
    if [ "$rc" -eq 0 ] && [ "$SMOKE" = "1" ]; then
      timeout 90s python3 scripts/smoke_navigation.py; rc=$?
      if [ "$rc" -eq 0 ] && [ -f scripts/smoke_teleop.py ]; then
        timeout 90s python3 scripts/smoke_teleop.py; rc=$?
      fi
    fi
    exit $rc
  ' ros_test "$@"
