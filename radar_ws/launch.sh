#! /bin/zsh

# rm -f ./user_logs/*
# touch ./user_logs/detect.log
# touch ./user_logs/serial.log

source /opt/ros/humble/setup.zsh
source install/setup.zsh

ros2 launch radar_bringup radar.launch.py