#!/bin/bash

echo "----- Start RealSense Setup -----"



echo "----- Install RealSense Library -----"

sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

echo "----- Install RealSense ROS2 Driver -----"

sudo apt install ros-humble-realsense2-*

echo "----- Finished setup RealSense -----"