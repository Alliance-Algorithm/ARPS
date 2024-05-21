#! /bin/zsh

echo "=======LAUNCH-PROGRESS=======" && \
# echo ">>>Refresh log files" && \
# rm -f ./resources/user_logs/*
# touch ./resources/user_logs/detect.log
# touch ./resources/user_logs/serial.log
# echo "[√]Done"

echo ">>>source /opt/ros/humble/setup.zsh" && \
source /opt/ros/humble/setup.zsh
echo "[√]Done"
echo ">>>source install/setup.zsh" && \
source install/setup.zsh
echo "[√]Done"

echo ">>>Run main application launch" && \
sleep 1s && \
ros2 launch radar_bringup radar.launch.py