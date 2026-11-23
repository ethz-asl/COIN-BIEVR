#!/bin/bash
set -eo pipefail
# Installs the Livox SDK2 and builds livox_ros_driver2 into the workspace, so that
# COIN-BIEVR is built with Livox CustomMsg support. Optional.
cd $COLCON_WS/src

# The official SDK2 does not support Ubuntu 24.04, so we use this fork.
git clone https://github.com/qza36/Livox-SDK2
mkdir -p Livox-SDK2/build && cd Livox-SDK2/build && cmake .. && make -j$(nproc) && sudo make install

cd $COLCON_WS/src
git clone https://github.com/Shahere/livox_ros_driver2.git
cd livox_ros_driver2
rm -f package.xml
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/

# Build the driver now, so a COIN-BIEVR code change does not rebuild it.
source /opt/ros/jazzy/setup.bash
cd $COLCON_WS
colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy --packages-select livox_ros_driver2
