#!/bin/bash

# run from root of the workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/local_setup.bash
# source install/setup.bash
ros2 run megatron ring_detector