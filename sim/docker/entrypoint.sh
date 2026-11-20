#!/usr/bin/env bash
# Source ROS, the prebuilt dependencies and this repository's packages (built on first use).
set -e
source /opt/ros/humble/setup.bash
source /opt/g1_deps/install/setup.bash
if [[ -d /ws/src/g1_navigation ]]; then
  if [[ ! -f /ws/build_sim/install/setup.bash || -n ${G1_SIM_REBUILD:-} ]]; then
    (cd /ws && colcon --log-base build_sim/log build --base-paths src --symlink-install \
       --build-base build_sim/build --install-base build_sim/install >/dev/null) \
      || { echo 'colcon build of /ws/src failed' >&2; exit 1; }
  fi
  source /ws/build_sim/install/setup.bash
fi
export PYTHONPATH=/ws/sim:${PYTHONPATH:-}
exec "$@"
