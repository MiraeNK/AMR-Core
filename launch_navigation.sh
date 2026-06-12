
#!/bin/bash

echo "[FMR] === Starting Navigation Mode ==="

echo "[FMR] Clearing previous processes..."



# Kill navigation/mapping tapi JANGAN lidar

pkill -f "navigation.launch.py" 2>/dev/null

pkill -f "mapping.launch.py" 2>/dev/null

pkill -f "slam_toolbox" 2>/dev/null

pkill -f "amcl" 2>/dev/null

pkill -f "controller_server" 2>/dev/null

pkill -f "planner_server" 2>/dev/null

sleep 2

echo "[FMR] Previous processes cleared."



# Cek apakah lidar sudah jalan

if ps aux | grep -q "lsc_laser_publisher" | grep -v grep; then

    echo "[FMR] LiDAR already running — skip start."

else

    echo "[FMR] LiDAR not running — starting ROS1 stack..."

    ~/catkin_ws/start_ros1_headless.sh &

    sleep 8

    echo "[FMR] ROS1 stack ready."

fi



# Cek bridge

if ! ps aux | grep "dynamic_bridge" | grep -v grep > /dev/null; then

    echo "[FMR] Bridge not running — starting..."

    source /opt/ros/noetic/setup.bash

    source ~/catkin_ws/devel/setup.bash

    source /opt/ros/foxy/setup.bash

    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &

    sleep 3

fi



echo "[FMR] Starting Navigation launch..."

source /opt/ros/foxy/setup.bash

source ~/amr_mp/install/setup.bash

ros2 launch amr_mp_bringup navigation.launch.py

