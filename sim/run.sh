#!/usr/bin/env bash
# Run the MuJoCo G1 simulator and the navigation stack (teleop, dry run) in Docker.
#   sim/run.sh [--scenario room|person|empty] [--check] [bringup.launch.py args...]
# Then, in another terminal on this PC: sim/pc.sh  (RViz + gamepad)
# --check runs sim/check_teleop.py against the running stack instead of waiting for you.
# ROS_DOMAIN_ID (default 42) is the stack's domain; the simulated robot publishes
# /utlidar/* on SIM_ROBOT_DOMAIN (default 41). All DDS traffic stays on localhost.
set -euo pipefail
cd "$(dirname "$0")/.."
SCENARIO=room CHECK=0
while [[ $# -gt 0 ]]; do
  case $1 in
    --scenario) SCENARIO=$2; shift 2 ;;
    --check) CHECK=1; shift ;;
    *) break ;;
  esac
done
[[ -d vendor/sim/unitree_rl_gym ]] || bash sim/fetch_assets.sh
docker image inspect g1-nav-sim >/dev/null 2>&1 || docker build -f sim/docker/Dockerfile -t g1-nav-sim .
TTY=(); [[ -t 0 ]] && TTY=(-it)
exec docker run --rm "${TTY[@]}" --network host --ipc host --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" -e ROS_LOCALHOST_ONLY=1 \
  -e SIM_ROBOT_DOMAIN="${SIM_ROBOT_DOMAIN:-41}" -e SCENARIO="$SCENARIO" -e CHECK="$CHECK" \
  -v "$PWD":/ws g1-nav-sim bash sim/docker/start_stack.sh "$@"
