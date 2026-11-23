#!/bin/bash
set -eo pipefail
# Creates and configures the catkin workspace and sources it in ~/.bashrc.
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
echo 'source $CATKIN_WS/devel/setup.bash' >> ~/.bashrc
