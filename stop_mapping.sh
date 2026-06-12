
#!/bin/bash

# Stop mapping tapi JANGAN matikan lidar + bridge

pkill -f "mapping.launch.py"

pkill -f "slam_toolbox"

pkill -f "async_slam_toolbox_node"

echo "Mapping stopped. Lidar still running."

