
#!/bin/bash

echo "[FMR] === Starting Mapping Mode ==="

echo "[FMR] Clearing previous processes..."



pkill -f "navigation.launch.py" 2>/dev/null

pkill -f "mapping.launch.py" 2>/dev/null

pkill -f "slam_toolbox" 2>/dev/null

pkill -f "amcl" 2>/dev/null

pkill -f "controller_server" 2>/dev/null

sleep 2

echo "[FMR] Previous processes cleared."



# Cek lidar

if ps aux | grep "lsc_laser_publisher" | grep -v grep > /dev/null; then

    echo "[FMR] LiDAR already running — skip start."

else

    echo "[FMR] LiDAR not running — starting ROS1 stack..."

    ~/catkin_ws/start_ros1_headless.sh &

    sleep 8

    echo "[FMR] ROS1 stack ready."

fi



echo "[FMR] Starting Mapping launch..."

source /opt/ros/foxy/setup.bash

source ~/amr_mp/install/setup.bash

ros2 launch amr_mp_bringup mapping.launch.py

