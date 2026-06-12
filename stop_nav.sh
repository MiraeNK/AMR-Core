
#!/bin/bash

# Stop navigation/mapping tapi JANGAN matikan lidar + bridge

pkill -f "navigation.launch.py"

pkill -f "mapping.launch.py"

pkill -f "nav2_bringup"

pkill -f "amcl"

pkill -f "map_server"

pkill -f "controller_server"

pkill -f "planner_server"

pkill -f "bt_navigator"

echo "Navigation stopped. Lidar still running."

