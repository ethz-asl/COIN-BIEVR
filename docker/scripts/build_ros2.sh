#!/bin/bash
set -eo pipefail
# Builds COIN-BIEVR from the source copied into $COLCON_WS/src/COIN-BIEVR.
source /opt/ros/jazzy/setup.bash
[ -f $COLCON_WS/install/setup.bash ] && source $COLCON_WS/install/setup.bash
cd $COLCON_WS
colcon build --packages-up-to coin_bievr_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
