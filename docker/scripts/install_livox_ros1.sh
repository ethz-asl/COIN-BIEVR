#!/bin/bash
set -eo pipefail
# Installs the Livox SDKs and builds both Livox ROS1 drivers into the workspace,
# so that COIN-BIEVR is built with Livox CustomMsg support. Optional.
cd $CATKIN_WS/src

# Livox SDK + livox_ros_driver (gen1)
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK/build && cmake .. && make -j$(nproc) && sudo make install
cd $CATKIN_WS/src
git clone https://github.com/livox-SDK/livox_ros_driver

# Livox SDK2 + livox_ros_driver2 (gen2)
git clone https://github.com/Livox-SDK/Livox-SDK2.git
mkdir -p Livox-SDK2/build && cd Livox-SDK2/build && cmake .. && make -j$(nproc) && sudo make install
cd $CATKIN_WS/src
git clone https://github.com/livox-SDK/livox_ros_driver2
cd livox_ros_driver2
rm -f package.xml
cp -f package_ROS1.xml package.xml

# Build the drivers now, so a COIN-BIEVR code change does not rebuild them.
cd $CATKIN_WS
catkin build livox_ros_driver livox_ros_driver2 -DROS_EDITION=ROS1
