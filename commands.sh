#!/bin/bash

# run from root of the workspace
source install/setup.bash
colcon build --symlink-install
ros2 run megatron ring_detector