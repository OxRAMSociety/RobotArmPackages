#!/usr/bin/env bash

# This file sets up the development environment for computer vision
# This only needs to be run once before starting to work on computer vision

# Make sure the submodules are installed
git submodule update --init --recursive

#Install usb camera drivers
sudo apt-get install ros-noetic-usb-cam

# Copy the calibration file to the correct place
mkdir -p ~/.ros/camera_info/
cp ./calibration.yaml ~/.ros/camera_info/head_camera.yaml

# Build darknet_ros in release mode
catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release
