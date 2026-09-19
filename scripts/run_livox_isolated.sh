#!/usr/bin/env bash
# Run the Livox MID360 driver+rviz isolated from any other ROS2 system on
# the LAN (e.g. a companion computer running mavros/PX4/Gazebo on the
# 192.168.10.0/24 network via enx00e04c364d05).
#
# Why this is needed: ROS_AUTOMATIC_DISCOVERY_RANGE defaults to SUBNET,
# so this Pi's ROS2 graph merges with any other ROS2 system reachable on
# any UP network interface. When that remote system is a busy mavros/PX4
# stack (~70 topics, high-rate mavlink/tf), the shared DDS traffic
# competes with the lidar driver and causes the point cloud to stutter/
# flicker (confirmed: rock-solid 10Hz only after isolating discovery).
#
# This only affects the process launched from this script — it does not
# change the default discovery range for any other terminal/session, so
# mavros connectivity elsewhere is unaffected.
set -euo pipefail

export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec ros2 launch livox_ros_driver2 rviz_MID360_launch.py
