#!/bin/bash
set -eo pipefail
# Creates the colcon workspace and sources ROS and the workspace in ~/.bashrc.
mkdir -p $COLCON_WS/src
echo '[ -f "/opt/ros/jazzy/setup.bash" ] && source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo '[ -f "${COLCON_WS}/install/setup.bash" ] && source ${COLCON_WS}/install/setup.bash' >> ~/.bashrc
