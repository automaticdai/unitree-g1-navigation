#!/bin/bash
# D435i colour preview for RViz. Runs ON the G1's PC2 (Ubuntu 20.04, ROS Foxy), e.g.
#   ssh g1-pc2 'ROS_DOMAIN_ID=42 bash -s' < scripts/pc2_camera.sh
# Publishes /camera/color/image_raw (640x480, 15 fps, colour only) into the
# navigation stack's domain. Preview only: nothing in the stack consumes it yet.
# Unitree's videohub service does not hold the camera while idle; if it starts
# streaming, it and this driver will compete for the RGB sensor.
set -e
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
# Foxy's CycloneDDS syntax; keep traffic on the robot's wired network.
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>eth0</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
exec ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/camera -r __node:=d435 \
  -p enable_color:=true -p enable_depth:=false -p enable_infra1:=false -p enable_infra2:=false \
  -p enable_gyro:=false -p enable_accel:=false -p rgb_camera.profile:=640x480x15 -p camera_name:=d435
