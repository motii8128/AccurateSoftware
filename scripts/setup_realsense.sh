#!/bin/bash

echo "----- Start RealSense Setup -----"

sudo apt update

echo "----- Register public key -----"

sudo mkdir -p /etc/apt/keyrings

curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \

sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update

echo "----- Install RealSense Library -----"

sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

echo "----- Install RealSense ROS2 Driver -----"

sudo apt install ros-humble-realsense2-*

echo "----- Finished setup RealSense -----"