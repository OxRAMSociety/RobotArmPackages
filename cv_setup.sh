#!/usr/bin/env bash

# This file sets up the development environment for computer vision
# This only needs to be run once before starting to work on computer vision

#Install usb camera drivers
sudo apt-get install ros-noetic-usb-cam

# Copy the calibration file to the correct place
mkdir -p ~/.ros/camera_info/
cp ./calibration.yaml ~/.ros/camera_info/head_camera.yaml
