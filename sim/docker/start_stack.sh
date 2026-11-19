#!/usr/bin/env bash
# Inside the container: start the simulated robot, then the stack as on the real G1
# (sensor_source:=utlidar, measured head mount), dry run, no bag.
set -euo pipefail
cd /ws
# The simulated mount is exactly the configured one, so calibration is confirmed here.
sed 's/calibration_confirmed: false/calibration_confirmed: true/' \
  src/g1_navigation/config/robot_g1_head.yaml > /tmp/robot_sim.yaml
python3 -m g1_sim.node --assets /ws/vendor/sim --scenario "$SCENARIO" \
  --source-domain-id "$SIM_ROBOT_DOMAIN" &
SIM=$!
ros2 launch g1_navigation bringup.launch.py mode:=teleop sensor_source:=utlidar \
  source_domain_id:="$SIM_ROBOT_DOMAIN" robot_config:=/tmp/robot_sim.yaml record_bag:=false "$@" &
STACK=$!
trap 'kill -TERM $STACK $SIM 2>/dev/null; wait' EXIT INT TERM
if [[ $CHECK == 1 ]]; then
  python3 sim/check_teleop.py --scenario "$SCENARIO"
else
  wait -n $SIM $STACK
fi
