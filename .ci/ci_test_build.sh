#!/bin/bash

set -e

mkdir -p ~/catkin_ws/src
cp -r $GITHUB_WORKSPACE/* ~/catkin_ws/src/
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make

echo "Ending build"