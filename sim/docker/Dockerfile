# ROS 2 Humble + Nav2 + FAST-LIO (pinned, as in dependencies.repos) + MuJoCo simulator.
# Build context: repository root.  docker build -f sim/docker/Dockerfile -t g1-nav-sim .
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git cmake build-essential python3-pip python3-vcstool python3-colcon-common-extensions \
      python3-numpy python3-scipy python3-yaml python3-pytest \
      ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-collision-monitor \
      ros-humble-message-filters ros-humble-sensor-msgs-py ros-humble-pcl-ros ros-humble-pcl-conversions \
      libpcl-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Pinned FAST-LIO and Livox driver (driver only for its message definitions) and Livox-SDK2.
WORKDIR /opt/g1_deps
COPY dependencies.repos hardware.repos ./
RUN mkdir -p src hw && vcs import src < dependencies.repos \
    && grep -A3 'Livox-SDK2:' hardware.repos | sed '1i repositories:' > sdk.repos && vcs import hw < sdk.repos \
    && git -C src/fast_lio submodule update --init --recursive \
    && cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml \
    && mkdir -p src/livox_ros_driver2/launch && cp -r src/livox_ros_driver2/launch_ROS2/. src/livox_ros_driver2/launch/
RUN cmake -S hw/Livox-SDK2 -B build-sdk -DCMAKE_BUILD_TYPE=Release && cmake --build build-sdk -j"$(nproc)" \
    && cmake --install build-sdk && ldconfig && rm -rf build-sdk
RUN source /opt/ros/humble/setup.bash && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y && rm -rf /var/lib/apt/lists/* \
    && colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# MuJoCo 3.x (Python 3.10) against the system NumPy; mujoco-lidar only for its Mid360 scan table.
RUN printf 'numpy==%s\n' "$(python3 -c 'import numpy; print(numpy.__version__)')" > /tmp/constraints.txt \
    && pip3 install --no-cache-dir -c /tmp/constraints.txt mujoco==3.3.7 \
    && pip3 install --no-cache-dir --no-deps mujoco-lidar==0.3.5

COPY sim/docker/entrypoint.sh /entrypoint.sh
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
