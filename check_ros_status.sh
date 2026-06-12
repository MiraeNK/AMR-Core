
#!/bin/bash

source /opt/ros/foxy/setup.bash 2>/dev/null

source ~/amr_mp/install/setup.bash 2>/dev/null



NODES=$(ros2 node list 2>/dev/null)

check_node() { echo "$NODES" | grep -q "$1" && echo "true" || echo "false"; }

check_proc() { ps aux | grep -v grep | grep -q "$1" && echo "true" || echo "false"; }



echo "{

  \"lidar\"      : $(check_proc 'lsc_laser_publisher'),

  \"roscore\"    : $(check_proc 'roscore'),

  \"ros1_bridge\": $(check_proc 'dynamic_bridge'),

  \"plc_driver\" : $(check_node '/plc_driver_node'),

  \"kinematics\" : $(check_node '/kinematics_node'),

  \"mqtt_bridge\": $(check_node '/amr_mqtt_bridge'),

  \"map_server\" : $(check_node '/map_server'),

  \"amcl\"       : $(check_node '/amcl'),

  \"nav2\"       : $(check_node '/controller_server'),

  \"slam\"       : $(check_node '/async_slam_toolbox_node'),

  \"timestamp\"  : $(date +%s)

}"

