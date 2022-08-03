#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros_ws/devel/setup.bash"

echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo ". /ros_ws/devel/setup.bash" >> ~/.bashrc

exec "$@"