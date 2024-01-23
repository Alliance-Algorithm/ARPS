#!/bin/bash

source /opt/ros/humble/setup.bash
cd radar_remap_cpp
source install/setup.bash
cd ../radar_yolov5_py
source install/setup.bash
cd ../bringup
ros2 launch allpkg.launch.py
