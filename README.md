# AMR MP — Complete System Documentation

**Project:** AMR Polebot — ROS 2 Architecture  
**Maintainer:** Hafizh Husaini \<miraenk7@gmail.com\>  
**Platform:** NVIDIA Jetson — Ubuntu 20.04 — ROS 2 Foxy  
**Version:** 1.2.0  
**Date:** May 2026

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Layer Architecture](#2-layer-architecture)
3. [Hardware](#3-hardware)
4. [Workspace Structure](#4-workspace-structure)
5. [Package Details](#5-package-details)
6. [Critical Parameters](#6-critical-parameters)
7. [TF Tree](#7-tf-tree)
8. [ROS 2 Topics](#8-ros-2-topics)
9. [How to Run the System](#9-how-to-run-the-system)
10. [Operation Modes](#10-operation-modes)
11. [Environment Setup](#11-environment-setup)
12. [Troubleshooting](#12-troubleshooting)
13. [Development Notes](#13-development-notes)

---

## 1. System Overview

AMR MP is an Autonomous Mobile Robot system built on ROS 2 Foxy, running on Polebot hardware. The system is designed with a layered modular architecture (microservices), allowing each component to be developed, replaced, or fixed independently without affecting other components.

### Design Philosophy

- **Single responsibility** — each node has one responsibility
- **Hardware isolation** — a single node is the only one allowed to communicate with the PLC
- **Parameter-driven** — all critical values are configured via YAML, not hardcoded
- **Safe by default** — all hardware safety mechanisms are active from startup
- **Scalable** — architecture is ready to be extended to fleet management
- **ros2_control ready** — C++ hardware interface is available for standard controller integration

### Hardware Constraints That Cannot Be Changed

The MC Protocol (SLMP) on the Mitsubishi PLC only allows **one active TCP connection** at a time. As a consequence, `plc_driver_node` must be the sole process holding a connection to the PLC.

---

## 2. Layer Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 5 — Software Interfacing                              │
│  amr_mp_teleop  (WASD keyboard — independent node)          │
│  Web Dashboard  (future — rosbridge + React)                 │
└─────────────────────────┬───────────────────────────────────┘
                          │ /cmd_vel
┌─────────────────────────▼───────────────────────────────────┐
│  Layer 4 — SLAM + Navigation                                 │
│  Mode A: slam_toolbox  (online_async — mapping)   ✓ TESTED  │
│  Mode B: slam_toolbox  (localization) + Nav2                 │
└─────────────────────────┬───────────────────────────────────┘
                          │ /map, /cmd_vel
┌─────────────────────────▼───────────────────────────────────┐
│  Layer 3 — Robot Description                                 │
│  amr_mp_description  (URDF xacro + TF static)               │
│  Convention: base_footprint (root) → base_link → chassis     │
│  TF: map→odom→base_footprint→base_link→laser_frame           │
└─────────────────────────┬───────────────────────────────────┘
                          │ /robot_description, TF
┌─────────────────────────▼───────────────────────────────────┐
│  Layer 2A — ros2_control (amr_mp_hardware)                   │
│  PoleBotHardwareInterface (C++) — AVAILABLE, not yet active  │
│  Pending: diff_drive_controller 0.9.0 parameter fix          │
│                                                              │
│  Layer 2B — kinematics_node (Python) ✓ ACTIVE              │
│  amr_base_controller — /odom + TF odom→base_footprint        │
│  /cmd_vel → /wheel_cmd_vel                                   │
└─────────────────────────┬───────────────────────────────────┘
                          │ /wheel_ticks, /wheel_cmd_vel
┌─────────────────────────▼───────────────────────────────────┐
│  Layer 1 — Hardware Abstraction              ✓ TESTED        │
│  amr_hardware_bridge/plc_driver_node                         │
│  SOLE MC Protocol connection to the PLC                      │
└─────────────────────────┬───────────────────────────────────┘
                          │ TCP MC Protocol
┌─────────────────────────▼───────────────────────────────────┐
│  PLC Mitsubishi  192.168.3.250:5007                          │
│  Encoder: D40-D141  |  Motor: D61, D161  |  M211, M220       │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  Layer 0 — ROS 1 Stack  (catkin_ws)          ✓ TESTED       │
│  lsc_ros_driver → /scan (ROS1) → dynamic_bridge → /scan(ROS2)│
│  NOTE: timestamp is offset by -0.1s for TF synchronization  │
│  File: catkin_ws/src/lsc_ros_driver/src/laser.cpp line 521   │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. Hardware

### Polebot Robot Specifications

| Component | Specification |
|---|---|
| Compute | NVIDIA Jetson (Ubuntu 20.04) |
| PLC | Mitsubishi Q-series |
| Motor Driver | Oriental Motor BLVD |
| LiDAR Sensor | Autonics LSC series (~15 Hz) |
| PLC Communication | MC Protocol (SLMP) TCP port 5007 |
| LiDAR Communication | TCP port 8000 |

### Mechanical Parameters (Source of Truth)

| Parameter | Value | Notes |
|---|---|---|
| `WHEEL_DIAMETER` | 0.110 m | Active wheel diameter |
| `WHEEL_SEPARATION` | 0.240 m | Distance between wheel center points |
| `WHEEL_RADIUS` | 0.055 m | = WHEEL_DIAMETER / 2 |
| `TICKS_PER_REV` | 360000.0 | Encoder ticks per full wheel revolution |
| `GEAR_RATIO` | 1.0 | Gearbox ratio from motor to wheel |

### PLC Registers

| Register | Function | Direction |
|---|---|---|
| D40, D41 | Left encoder (lo, hi) | Read |
| D140, D141 | Right encoder (lo, hi) | Read |
| D61, D62 | Right motor command (lo, hi) | Write |
| D161, D162 | Left motor command (lo, hi) | Write |
| M211 | Heartbeat toggle bit | Write |
| M212 | Enable bit (always 1) | Write |
| M220 | Servo lock (1=active, 0=release) | Write |

### Motor Command Value Convention

```
+100  → full speed forward
-100  → full speed reverse
Range: -100 to +100 (PLC units, 32-bit signed)

Format: 2 x 16-bit words (lo_word, hi_word)
+100 → [100,  0]  (0x00000064)
-100 → [-100, -1] (0xFFFFFF9C)
```

### Wheel Orientation Convention

Standard differential drive — no negation. Formula:
```
delta_theta = (dr_dist - dl_dist) / wheel_separation
```
Physical right turn → negative `delta_theta` → decreasing yaw → correct.

---

## 4. Workspace Structure

### ROS 1 Workspace — `~/catkin_ws`

```
catkin_ws/
├── src/
│   └── lsc_ros_driver/
│       ├── launch/
│       │   └── lsc_c25_launch.launch    ← frame_id: laser_frame
│       └── src/
│           └── laser.cpp                ← timestamp -0.1s offset (line 521)
├── start_ros1_stack.sh                  ← Layer 0 startup script
└── amr_monitor.py                       ← Live dashboard monitor
```

**IMPORTANT — laser.cpp modification:**
```cpp
// Line 521 — modified for ROS 2 TF synchronization
p_laser_scan->header.stamp = ros::Time::now() - ros::Duration(0.1);
// Backup stored at: laser.cpp.bak
```

### ROS 2 Workspace — `~/amr_mp`

```
amr_mp/
├── src/
│   ├── amr_hardware_bridge/           ← Layer 1 (Python)
│   │   ├── amr_hardware_bridge/
│   │   │   └── plc_driver_node.py
│   │   └── config/
│   │       └── plc_driver_params.yaml
│   │
│   ├── amr_base_controller/           ← Layer 2B active (Python)
│   │   ├── amr_base_controller/
│   │   │   └── kinematics_node.py
│   │   └── config/
│   │       └── kinematics_params.yaml
│   │
│   ├── amr_mp_description/            ← Layer 3 (CMake)
│   │   ├── description/
│   │   │   ├── robot.urdf.xacro
│   │   │   ├── robot_core.xacro       ← base_footprint as root frame
│   │   │   ├── lidar.xacro
│   │   │   ├── ros2_control.xacro
│   │   │   └── inertial_macros.xacro
│   │   └── launch/
│   │       └── rsp.launch.py
│   │
│   ├── amr_mp_hardware/               ← Layer 2A ros2_control (C++)
│   │   ├── include/amr_mp_hardware/
│   │   │   └── polebot_hardware_interface.hpp
│   │   ├── src/
│   │   │   └── polebot_hardware_interface.cpp
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── amr_mp_hardware_plugin.xml
│   │
│   ├── amr_mp_teleop/                 ← Layer 5 (Python, independent)
│   │   ├── amr_mp_teleop/
│   │   │   └── manual_control_node.py
│   │   └── config/
│   │       └── teleop_params.yaml
│   │
│   └── amr_mp_bringup/                ← Bringup orchestration (CMake)
│       ├── launch/
│       │   ├── mapping.launch.py
│       │   └── navigation.launch.py
│       └── config/
│           ├── controllers.yaml
│           ├── slam_toolbox_params.yaml
│           ├── slam_toolbox_localization_params.yaml
│           └── nav2_params.yaml
│
└── mapping.rviz                        ← RViz2 config for mapping
```

---

## 5. Package Details

### 5.1 `amr_hardware_bridge` — Layer 1

**Type:** Python (ament_python)  
**Node:** `plc_driver_node`  
**Frequency:** 10 Hz  
**Status:** ✓ Tested and verified

**Responsibilities:**
- Sole TCP connection to Mitsubishi PLC via pymcprotocol
- Read encoders D40–D141 in a single batchread (102 words, single-packet)
- Write motor commands D61/D161
- Send heartbeat M211 and servo lock M220
- Encoder jump filter (delta > 0.20 m per cycle is ignored)
- Zero-crossing cooldown: 0.5 seconds
- Auto-reconnect with 2.0-second backoff
- Safe shutdown: motors are zeroed before the connection is closed

**Published Topics:**

| Topic | Type | Content |
|---|---|---|
| `/wheel_ticks` | `Int64MultiArray` | `[left_u32, right_u32]` |
| `/plc_status` | `Bool` | `True` = connected |

**Subscribed Topics:**

| Topic | Type | Content |
|---|---|---|
| `/wheel_cmd_vel` | `Float64MultiArray` | `[right_speed, left_speed]` range -100..100 |

---

### 5.2 `amr_mp_hardware` — Layer 2A (ros2_control C++)

**Type:** C++ (ament_cmake)  
**Plugin:** `amr_mp_hardware/PoleBotHardwareInterface`  
**Status:** Build OK, pending activation in bringup

**Note:** `diff_drive_controller` version 0.9.0 on Foxy has a bug reading parameters from external YAML files. Layer 2B (`kinematics_node`) is currently used as a replacement. Layer 2A will be activated once the controller bug is resolved.

**Foxy API Methods:**

| Method | Function |
|---|---|
| `configure()` | Load parameters from URDF, initialize arrays |
| `export_state_interfaces()` | Expose position + velocity per joint |
| `export_command_interfaces()` | Expose velocity command per joint |
| `start()` | Create internal ROS node, subscriber, publisher |
| `stop()` | Send stop, halt spin thread |
| `read()` | State updated asynchronously by ticks_callback |
| `write()` | Convert rad/s → PLC speed, publish /wheel_cmd_vel |

---

### 5.3 `amr_base_controller` — Layer 2B (Active)

**Type:** Python (ament_python)  
**Node:** `kinematics_node`  
**Status:** ✓ Actively used in mapping, orientation verified correct

**Differential drive kinematics formula:**

```python
# Delta ticks (u32 wrapping-safe)
dl_u32   = (left_u32  - last_left)  & 0xFFFFFFFF
dl_ticks = signed(dl_u32)
dl_dist  = (dl_ticks / ticks_per_rev / gear_ratio) * circumference

# Standard kinematics — WITHOUT negation
delta_s     = (dl_dist + dr_dist) * 0.5
delta_theta = (dr_dist - dl_dist) / wheel_separation

# Mid-yaw integration (1st-order Runge-Kutta)
mid_yaw = yaw + 0.5 * delta_theta
x   += delta_s * cos(mid_yaw)
y   += delta_s * sin(mid_yaw)
yaw += delta_theta
```

**cmd_vel → wheel_cmd_vel conversion:**

```python
v_right = linear.x + angular.z * (wheel_separation / 2.0)
v_left  = linear.x - angular.z * (wheel_separation / 2.0)
speed_right = (v_right / max_linear_vel) * speed_max
speed_left  = (v_left  / max_linear_vel) * speed_max
```

---

### 5.4 `amr_mp_description` — Layer 3

**Type:** CMake (ament_cmake)  
**Status:** ✓ TF tree verified

**Correct frame convention:**
```
base_footprint (ROOT — kinematics_node broadcasts here)
    └── base_link
         └── chassis
              ├── laser_frame
              ├── caster_wheel
         ├── left_wheel
         └── right_wheel
```

**IMPORTANT:** `base_footprint` must be the root (parent) and `base_link` the child. If reversed, a TF conflict "two or more unconnected trees" will occur.

**LiDAR position relative to chassis:**
- X: +0.25 m (front)
- Z: +0.175 m (up)
- RPY: `0 0 -1.5708` (requires physical verification)

---

### 5.5 `amr_mp_teleop` — Layer 5

**Type:** Python (ament_python)  
**Node:** `manual_control_node`  
**Status:** ✓ Tested, WASD working correctly

**Controls:**

| Key | Action |
|---|---|
| W | Forward |
| S | Reverse |
| A | Turn left |
| D | Turn right |
| Q | Quit |

**Published Topic:** `/cmd_vel` (geometry_msgs/Twist)

---

### 5.6 `amr_mp_bringup` — Orchestration

**Launch files:**

**`mapping.launch.py`** — Mode A (✓ Tested):
```
RSP + plc_driver_node + kinematics_node +
joint_state_publisher + slam_toolbox (delay 3s) + manual_control_node
```

**`navigation.launch.py`** — Mode B (not yet tested):
```
RSP + plc_driver_node + kinematics_node +
slam_toolbox (localization) + Nav2
```

---

## 6. Critical Parameters

### `plc_driver_params.yaml`

```yaml
plc_driver_node:
  ros__parameters:
    plc_host: '192.168.3.250'
    plc_port: 5007
    cycle_hz: 10.0
    reconnect_s: 2.0
    wheel_diameter_m: 0.110
    wheel_separation_m: 0.240
    ticks_per_rev: 360000.0
    gear_ratio: 1.0
    speed_max: 100
    speed_min: -100
    zero_crossing_cooldown_s: 0.50  # DO NOT set below 0.4
    jump_filter_m: 0.20
    enc_head: 'D40'
    enc_read_size: 102
    cmd_right_head: 'D61'
    cmd_left_head: 'D161'
    heartbeat_head: 'M211'
    servo_lock_head: 'M220'
```

### `kinematics_params.yaml`

```yaml
kinematics_node:
  ros__parameters:
    wheel_diameter_m: 0.110
    wheel_separation_m: 0.240
    ticks_per_rev: 360000.0
    gear_ratio: 1.0
    odom_frame_id: 'odom'
    base_frame_id: 'base_footprint'
    max_linear_vel: 0.5       # m/s — field calibration required
    max_angular_vel: 1.0      # rad/s — field calibration required
    speed_max: 100
    speed_min: -100
    jump_filter_m: 0.20
```

### `slam_toolbox_params.yaml` (critical parameters)

```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping
    transform_timeout: 1.0      # increased to tolerate ROS1 bridge delay
    tf_buffer_duration: 60.0    # increased
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 20.0
```

---

## 7. TF Tree

```
map
 └── odom                          ← from slam_toolbox
      └── base_footprint            ← from kinematics_node (10 Hz)
           └── base_link            ← from robot_state_publisher (URDF static)
                ├── chassis
                │    ├── caster_wheel
                │    └── laser_frame    ← frame_id for /scan LiDAR
                ├── left_wheel
                └── right_wheel
```

**Broadcast rates:**
- `odom → base_footprint` : 10 Hz (kinematics_node)
- `base_link → laser_frame` : static from URDF
- `map → odom` : corrected by slam_toolbox on loop closure

---

## 8. ROS 2 Topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/wheel_ticks` | `Int64MultiArray` | `plc_driver_node` | `kinematics_node` |
| `/plc_status` | `Bool` | `plc_driver_node` | — |
| `/wheel_cmd_vel` | `Float64MultiArray` | `kinematics_node` | `plc_driver_node` |
| `/odom` | `nav_msgs/Odometry` | `kinematics_node` | `slam_toolbox` |
| `/cmd_vel` | `geometry_msgs/Twist` | `manual_control_node` / Nav2 | `kinematics_node` |
| `/scan` | `sensor_msgs/LaserScan` | `ros1_bridge` (~15 Hz) | `slam_toolbox` |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` (0.2 Hz) | Nav2, RViz2 |
| `/robot_description` | `String` | `robot_state_publisher` | all |
| `/tf` | `tf2_msgs/TFMessage` | `kinematics_node`, RSP | all |
| `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` | all |

---

## 9. How to Run the System

### Prerequisites

```bash
ping 192.168.3.250   # PLC must reply
ping 192.168.3.30    # LiDAR must reply
```

### Step 1 — Layer 0 (always first)

```bash
cd ~/catkin_ws
./start_ros1_stack.sh
```

Verify /scan:
```bash
ros2 topic hz /scan   # should be ~15 Hz
```

### Step 2A — Mapping Mode

```bash
# Terminal 1
ros2 launch amr_mp_bringup mapping.launch.py

# Terminal 2 — RViz2
rviz2 -d ~/amr_mp/mapping.rviz

# Terminal 3 — manual control
ros2 run amr_mp_teleop manual_control_node \
  --ros-args --params-file ~/amr_mp/src/amr_mp_teleop/config/teleop_params.yaml
```

Drive the robot around the area using WASD. When done, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/gudang
```

### Step 2B — Navigation Mode

```bash
ros2 launch amr_mp_bringup navigation.launch.py map:=~/maps/gudang.yaml
```

### Manual Control Only

```bash
# Terminal 1
ros2 run amr_hardware_bridge plc_driver_node \
  --ros-args --params-file ~/amr_mp/src/amr_hardware_bridge/config/plc_driver_params.yaml

# Terminal 2
ros2 run amr_base_controller kinematics_node \
  --ros-args --params-file ~/amr_mp/src/amr_base_controller/config/kinematics_params.yaml

# Terminal 3
ros2 run amr_mp_teleop manual_control_node \
  --ros-args --params-file ~/amr_mp/src/amr_mp_teleop/config/teleop_params.yaml
```

---

## 10. Operation Modes

### Mode A — Mapping (✓ Tested)

```
Goal       : Build a warehouse area map
Controller : kinematics_node
Control    : Manual WASD
SLAM       : slam_toolbox online_async
/map rate  : 0.2 Hz (update every 5 seconds — normal)
Output     : ~/maps/gudang.pgm + ~/maps/gudang.yaml
```

### Mode B — Navigation (not yet tested)

```
Goal       : Daily autonomous operation
Controller : kinematics_node
Localization: slam_toolbox localization mode
Navigation : Nav2 DWB planner
Goal input : Via RViz2 2D Nav Goal or /goal_pose
```

### Mode Comparison

| Aspect | Mode A | Mode B |
|---|---|---|
| slam_toolbox | online_async | localization |
| Map | Built in real-time | Loaded from static file |
| Control | Manual WASD | Nav2 autonomous |
| /map update | 0.2 Hz | not updated |

---

## 11. Environment Setup

### Correct `.bashrc` Configuration

```bash
source /opt/ros/foxy/setup.bash
source ~/amr_ws/install/setup.bash

# AMR_MP workspace
export AMENT_PREFIX_PATH=/home/eqdev/amr_mp/install/amr_hardware_bridge:/home/eqdev/amr_mp/install/amr_base_controller:/home/eqdev/amr_mp/install/amr_mp_description:/home/eqdev/amr_mp/install/amr_mp_teleop:/home/eqdev/amr_mp/install/amr_mp_bringup:/home/eqdev/amr_mp/install/amr_mp_hardware:$AMENT_PREFIX_PATH
export PATH=/home/eqdev/amr_mp/install/amr_hardware_bridge/lib/amr_hardware_bridge:/home/eqdev/amr_mp/install/amr_base_controller/lib/amr_base_controller:/home/eqdev/amr_mp/install/amr_mp_teleop/lib/amr_mp_teleop:$PATH
export PYTHONPATH=/home/eqdev/amr_mp/install/amr_hardware_bridge/lib/python3.8/site-packages:/home/eqdev/amr_mp/install/amr_base_controller/lib/python3.8/site-packages:/home/eqdev/amr_mp/install/amr_mp_teleop/lib/python3.8/site-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/home/eqdev/amr_mp/install/amr_mp_hardware/lib:$LD_LIBRARY_PATH
```

### Per-Package Build

```bash
cd ~/amr_mp
colcon build --packages-select amr_hardware_bridge --symlink-install
colcon build --packages-select amr_base_controller --symlink-install
colcon build --packages-select amr_mp_hardware --symlink-install
colcon build --packages-select amr_mp_description --symlink-install
colcon build --packages-select amr_mp_teleop --symlink-install
colcon build --packages-select amr_mp_bringup --symlink-install
```

### ROS 1 Build (if LiDAR driver is modified)

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg lsc_ros_driver
```

---

## 12. Troubleshooting

### slam_toolbox keeps dropping laser_frame

```
Symptom   : Message Filter dropping message: frame 'laser_frame'
Root cause: /scan timestamp from ROS 1 is newer than the TF buffer

Already fixed in:
  catkin_ws/src/lsc_ros_driver/src/laser.cpp line 521
  p_laser_scan->header.stamp = ros::Time::now() - ros::Duration(0.1);

If the issue returns:
  - Increase transform_timeout in slam_toolbox_params.yaml
  - Check whether laser.cpp.bak still exists (backup before modification)
  - Ensure catkin_make has been run after editing laser.cpp
```

### TF "two or more unconnected trees"

```
Symptom   : Could not find connection between odom and base_footprint
Root cause: URDF defines base_link as parent of base_footprint,
            while kinematics_node broadcasts odom→base_footprint

Solution  : base_footprint must be ROOT in the URDF
            Joint: base_footprint → base_link (not the reverse)
            Already fixed in robot_core.xacro
```

### Robot orientation is reversed (left-right)

```
Symptom   : Press D (right) but robot in RViz turns left
Root cause: Unnecessary negation of delta_theta

Solution  : Use standard formula without negation:
            delta_theta = (dr_dist - dl_dist) / wheel_separation
            Already fixed in kinematics_node.py
```

### diff_drive_controller "Wheel names parameters are empty"

```
Symptom   : ros2_control_node crash exit code -6
Root cause: Bug in diff_drive_controller 0.9.0 on Foxy
            Does not read parameter list from external YAML file

Workaround: Use kinematics_node (Layer 2B) as a replacement
            Layer 2A (ros2_control) is available but not activated
```

### PLC not connecting

```
Check: ping 192.168.3.250
       Ensure no other program is connected to the PLC
       Ensure PLC is in RUN state
```

### /scan not appearing in ROS 2

```
Check: Ensure start_ros1_stack.sh is running
       ping 192.168.3.30
       Check the ros1_bridge terminal
```

### amr_mp package not detected

```
Check: cat ~/.bashrc | grep amr_mp
       Open a new terminal after editing .bashrc
       ls ~/amr_mp/install/
```

---

## 13. Development Notes

### Changes from v1.0.0 to v1.1.0

- Added `amr_mp_hardware` (ros2_control C++ hardware interface)
- Updated launch files for ros2_control
- Updated TF tree documentation

### Changes from v1.1.0 to v1.2.0

- **Fix URDF TF convention** — `base_footprint` made root frame (parent), `base_link` becomes child. This is a critical fix that resolved "two or more unconnected trees"
- **Fix LiDAR timestamp** — `laser.cpp` line 521 now subtracts `-ros::Duration(0.1)` for ROS 2 TF buffer synchronization
- **Fix odometry orientation** — Removed `delta_theta` negation in `kinematics_node.py`. Standard differential drive formula is correct without negation
- **Mapping successfully tested** — `/map` publishes at 0.2 Hz, SLAM active, orientation 100% matches physical robot
- **slam_toolbox params** — `transform_timeout` increased to 1.0s, `tf_buffer_duration` to 60s
- **mapping.launch.py** — switched from ros2_control to kinematics_node (workaround for diff_drive_controller bug)
- **Added mapping.rviz** — ready-to-use RViz2 config for mapping sessions
- **manual_control_node** — rewritten, fully functional with WASD

### Hardware Test Status

| Component | Status |
|---|---|
| PLC connection | ✓ Verified |
| /wheel_ticks 10 Hz | ✓ Verified |
| /odom 10 Hz | ✓ Verified |
| Full TF tree | ✓ Verified |
| /scan 15 Hz from bridge | ✓ Verified |
| SLAM mapping | ✓ Tested, /map publishing |
| Left/right orientation | ✓ 100% matches physical robot |
| WASD control | ✓ Verified |
| Navigation Mode | Not yet tested |
| Map saving | Not yet tested |

### Still Needs Calibration

- **LiDAR position and orientation** — `lidar.xacro` rpy `0 0 -1.5708` requires physical verification
- **`max_linear_vel`** — measure actual distance traveled at speed=100 for 1 second
- **`max_angular_vel`** — measure actual rotation angle
- **Nav2 `robot_radius`** — measure chassis diagonal divided by two

### Next Test Sequence

```
1. ✓ Mapping successful
2. → Save map: ros2 run nav2_map_server map_saver_cli -f ~/maps/gudang
3. → Test navigation.launch.py with saved map
4. → Calibrate max_linear_vel and max_angular_vel
5. → Verify LiDAR position in lidar.xacro
6. → Activate ros2_control (Layer 2A) after diff_drive_controller fix
```

### Development Roadmap

```
Next phases:
├── Web Dashboard (rosbridge + React)
├── Fleet Management (multi-robot)
└── Docker containerization
```

### Critical Dependencies

```
Python 3.8     — ROS 2 Foxy default
pymcprotocol   — Mitsubishi PLC communication
rclpy          — ROS 2 Python client
rclcpp         — ROS 2 C++ client
hardware_interface — ros2_control (Foxy API)
slam_toolbox   — SLAM and localization
```

### ros2_control API Differences: Foxy vs Humble

| Aspect | Foxy | Humble |
|---|---|---|
| Init | `configure()` | `on_init()` |
| Activate | `start()` | `on_activate()` |
| Deactivate | `stop()` | `on_deactivate()` |
| Return type | `return_type::OK` | `CallbackReturn::SUCCESS` |
| info_ member | manually declared | from base class |
| diff_drive wheel param | bug in 0.9.0 | `left_wheel_names` list |

---

*Documentation version 1.2.0 — May 2026*  
*Mapping successfully tested on Polebot hardware.*
