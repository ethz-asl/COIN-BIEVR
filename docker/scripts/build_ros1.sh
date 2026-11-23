#!/bin/bash
set -eo pipefail
# Builds COIN-BIEVR from the source copied into $CATKIN_WS/src/COIN-BIEVR.
cd $CATKIN_WS
catkin build coin_bievr_ros -DROS_EDITION=ROS1
