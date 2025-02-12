#!/bin/bash

echo "----- Start YD-Lidar SDK Setup -----"

echo "----- Install CMake -----"

sudo apt install cmake pkg-config

cd /home/$USER/

echo "----- Clone YDLidar-SDK -----"

git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd ./YDLidar-SDK

mkdir build
cd build

echo "----- Start Build -----"

cmake ..

make

echo "----- Install SDK -----"

sudo make install

echo "----- Finished setup YD Lidar -----"