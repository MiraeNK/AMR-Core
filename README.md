
# AMR MP — Complete System Documentation

**Project:** AMR Polebot — ROS 2 Architecture  
**Maintainer:** Hafizh Husaini <miraenk7@gmail.com>  
**Platform:** NVIDIA Jetson — Ubuntu 20.04 — ROS 2 Foxy  
**Version:** 1.3.0  
**Date:** June 2026

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

As of version 1.3.0, the system is **100% Native ROS 2**, completely eliminating the legacy ROS 1 bridge and scan relays for zero-latency sensor processing.

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

```text
┌─────────────────────────────────────────────────────────────┐
│  Layer 5 — Software Interfacing                             │
│  amr_mp_teleop  (WASD keyboard — independent node)          │
│  Web Dashboard  (future — rosbridge + React)                │
└─────────────────────────┬───────────────────────────────────┘
                          │ /cmd_vel
┌─────────────────────────▼───────────────────────────────────┐
│  Layer 4 — SLAM + Navigation                                │
│  Mode A: slam_toolbox  (online_async — mapping)    ✓ TESTED │
│  Mode B: slam_toolbox  (localization) + Nav2                │
└─────────────────────────┬───────────────────────────────────┘
                          │ /map, /cmd_vel
┌─────────────────────────▼───────────────────────────────────┐
│  Layer 3 — Robot Description                                │
│  amr_mp_description  (URDF xacro + TF static)               │
│  Convention: base_footprint (root) → base_link → chassis    │
│  TF: map→odom→base_footprint→base_link→laser_frame          │
└─────────────────────────┬───────────────────────────────────┘
                          │ /robot_description, TF
┌─────────────────────────▼───────────────────────────────────┐
│  Layer 2A — ros2_control (amr_mp_hardware)                  │
│  PoleBotHardwareInterface (C++) — AVAILABLE                 │
│  Pending: diff_drive_controller 0.9.0 parameter fix         │
│                                                             │
│  Layer 2B — kinematics_node (Python) ✓ ACTIVE               │
│  amr_base_controller — /odom + TF odom→base_footprint       │
│  /cmd_vel → /wheel_cmd_vel                                  │
└─────────────────────────┬───────────────────────────────────┘
                          │ /wheel_ticks, /wheel_cmd_vel
┌─────────────────────────▼───────────────────────────────────┐
│  Layer 1 — Hardware Abstraction               ✓ TESTED      │
│  amr_hardware_bridge/plc_driver_node (TCP 5007)             │
│  sick_scan_xd (Native SICK LiDAR driver - TCP 2112)         │
└─────────────────────────┬───────────────────────────────────┘
                          │ Native Ethernet Protocols
┌─────────────────────────▼───────────────────────────────────┐
│  [PLC Mitsubishi: 192.168.3.250] | [SICK LiDAR: 192.168.3.x]│
└─────────────────────────────────────────────────────────────┘

```

---

## 3. Hardware

### Polebot Robot Specifications

| Component | Specification |
| --- | --- |
| Compute | NVIDIA Jetson (Ubuntu 20.04) |
| PLC | Mitsubishi Q-series / iQ-R |
| Motor Driver | Oriental Motor BLVD |
| LiDAR Sensor | SICK LiDAR (TiM series via `sick_scan_xd`) |
| PLC Communication | MC Protocol (SLMP) TCP port 5007 |
| LiDAR Communication | Native Ethernet TCP port 2111/2112 |

### Mechanical Parameters (Source of Truth)

| Parameter | Value | Notes |
| --- | --- | --- |
| `WHEEL_DIAMETER` | 0.110 m | Active wheel diameter |
| `WHEEL_SEPARATION` | 0.240 m | Distance between wheel center points |
| `WHEEL_RADIUS` | 0.055 m | = WHEEL_DIAMETER / 2 |
| `TICKS_PER_REV` | 360000.0 | Encoder ticks per full wheel revolution |
| `GEAR_RATIO` | 1.0 | Gearbox ratio from motor to wheel |

### PLC Registers

| Register | Function | Direction |
| --- | --- | --- |
| D40, D41 | Left encoder (lo, hi) | Read |
| D140, D141 | Right encoder (lo, hi) | Read |
| D61, D62 | Right motor command (lo, hi) | Write |
| D161, D162 | Left motor command (lo, hi) | Write |
| M211 | Heartbeat toggle bit | Write |
| M212 | Enable bit (always 1) | Write |
| M220 | Servo lock (1=active, 0=release) | Write |

---

## 4. Workspace Structure

### ROS 2 Workspace — `~/amr_mp`

```text
amr_mp/
├── src/
│   ├── amr_hardware_bridge/           ← Layer 1 (PLC Python)
│   ├── sick_scan_xd/                  ← Layer 1 (SICK LiDAR C++)
│   │
│   ├── amr_base_controller/           ← Layer 2B active (Kinematics)
│   │
│   ├── amr_mp_description/            ← Layer 3 (URDF/Xacro)
│   │   ├── description/
│   │   │   ├── robot.urdf.xacro
│   │   │   ├── robot_core.xacro       ← base_footprint as root
│   │   │   └── lidar.xacro            ← SICK Lidar physical config
│   │   └── launch/rsp.launch.py
│   │
│   ├── amr_mp_hardware/               ← Layer 2A ros2_control (C++)
│   │
│   ├── amr_mp_teleop/                 ← Layer 5 (WASD Manual Control)
│   │
│   └── amr_mp_bringup/                ← Bringup orchestration (CMake)
│       ├── launch/
│       │   ├── mapping.launch.py      ← Native ROS 2 mapping
│       │   └── navigation.launch.py
│       └── config/
│           ├── nav2_params.yaml
│           └── slam_toolbox_params.yaml
│
└── maps/                              ← Directory for generated PGM/YAML maps

```

---

## 5. Package Details

### 5.1 `amr_hardware_bridge` — Layer 1

**Type:** Python (ament_python)

**Node:** `plc_driver_node`

**Frequency:** 10 Hz

**Status:** ✓ Tested and verified (Atomic Write)

**Responsibilities:**

* Sole TCP connection to Mitsubishi PLC via `pymcprotocol`.
* Uses **Atomic Batch Write** to ensure Left & Right motor speeds enter PLC memory simultaneously to prevent jerky rotations.
* Auto-reconnect with 2.0-second backoff and safe motor zeroing on shutdown.

### 5.2 `sick_scan_xd` — Layer 1 (LiDAR)

**Type:** C++ (ament_cmake)

**Status:** ✓ Tested and integrated

* Replaces the old ROS 1 Autonics stack. Directly reads native Ethernet data from SICK LiDAR and publishes zero-latency `sensor_msgs/LaserScan` to `/scan`.

### 5.3 `amr_base_controller` — Layer 2B (Active)

**Type:** Python (ament_python)

**Node:** `kinematics_node`

**Status:** ✓ Actively used in mapping, orientation verified correct

**Standard kinematics — WITHOUT negation:**

```python
delta_s     = (dl_dist + dr_dist) * 0.5
delta_theta = (dr_dist - dl_dist) / wheel_separation

```

*(Physical right turn → negative `delta_theta` → decreasing yaw → correct).*

### 5.4 `amr_mp_description` — Layer 3

**Type:** CMake (ament_cmake)

**Status:** ✓ TF tree verified

**Correct frame convention:**
`base_footprint` must be the root (parent) and `base_link` the child. Resolves the "two or more unconnected trees" TF conflict.

### 5.5 `amr_mp_bringup` — Orchestration

**`mapping.launch.py`** — Mode A (✓ Tested):

```text
RSP + plc_driver_node + kinematics_node + sick_scan_xd + slam_toolbox + manual_control_node

```

---

## 6. Critical Parameters

### `kinematics_params.yaml`

```yaml
kinematics_node:
  ros__parameters:
    wheel_diameter_m: 0.110
    wheel_separation_m: 0.240
    ticks_per_rev: 360000.0
    max_linear_vel: 0.5       # m/s
    max_angular_vel: 1.0      # rad/s

```

### `slam_toolbox_params.yaml`

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: false
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping

```

---

## 7. TF Tree

```text
map
 └── odom                         ← from slam_toolbox
      └── base_footprint          ← from kinematics_node (10 Hz)
            └── base_link         ← from robot_state_publisher (URDF static)
                ├── chassis
                │   ├── caster_wheel
                │   └── laser_frame    ← frame_id for SICK /scan
                ├── left_wheel
                └── right_wheel

```

---

## 8. ROS 2 Topics

| Topic | Type | Publisher | Subscriber |
| --- | --- | --- | --- |
| `/wheel_ticks` | `Int64MultiArray` | `plc_driver_node` | `kinematics_node` |
| `/wheel_cmd_vel` | `Float64MultiArray` | `kinematics_node` | `plc_driver_node` |
| `/odom` | `nav_msgs/Odometry` | `kinematics_node` | `slam_toolbox` / Nav2 |
| `/cmd_vel` | `geometry_msgs/Twist` | `teleop` / Nav2 | `kinematics_node` |
| `/scan` | `sensor_msgs/LaserScan` | `sick_scan_xd` | `slam_toolbox` / Nav2 |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | Nav2 / RViz2 |

---

## 9. How to Run the System

### Prerequisites

Ensure PLC and LiDAR are reachable via network:

```bash
ping 192.168.3.250   # PLC
ping 192.168.3.xxx   # SICK LiDAR

```

### Mapping Mode (SLAM)

```bash
# Terminal 1 - Bringup Base & Lidar
ros2 launch amr_mp_bringup robot_base.launch.py

# Terminal 2 - Mapping Node
ros2 launch amr_mp_bringup mapping.launch.py

# Terminal 3 - RViz2
rviz2 -d ~/amr_mp/src/amr_mp_bringup/config/navigation.rviz

# Terminal 4 - Manual Control
ros2 run amr_mp_teleop manual_control_node

```

When mapping is complete, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/gudang_baru

```

---

## 10. Operation Modes

| Aspect | Mode A (Mapping) | Mode B (Navigation) |
| --- | --- | --- |
| **Goal** | Build warehouse area map | Daily autonomous operation |
| **SLAM Mode** | `online_async` | `localization` |
| **Control** | Manual WASD | Nav2 (DWB Planner) |
| **Map Source** | Built in real-time | Loaded from `gudang_baru.yaml` |

---

## 11. Environment Setup

### Workspace Build

```bash
cd ~/amr_mp
colcon build --symlink-install
source install/setup.bash

```

---

## 12. Troubleshooting

### PLC `<ES:0180840b>` Socket Error

**Symptom:** PLC driver refuses to connect despite successful ping.
**Solution:** The PLC socket is stuck in a Time-Wait state from a previous crash. Power cycle the PLC for 5 seconds to flush the socket memory.

### SLAM stops processing /scan (Freezes)

**Symptom:** Map is blank, or stops expanding.
**Solution:** Ensure `use_sim_time` is strictly set to `false` in both `mapping.launch.py` and `slam_toolbox_params.yaml`.

### TF "two or more unconnected trees"

**Symptom:** `odom` is not linked to `base_link`.
**Solution:** Double-check `base_frame_id` inside `kinematics_params.yaml` matches exactly the ROOT frame of the URDF (`base_footprint`).

---

## 13. Development Notes

### Changes in v1.3.0

* **Native ROS 2 LiDAR:** Ripped out the entire legacy ROS 1 `catkin_ws`, `lsc_ros_driver`, and `ros1_bridge`. The SICK LiDAR now communicates directly with ROS 2 via `sick_scan_xd`.
* **Latency Fix:** The `-0.1s` TF offset hack is no longer needed since the LiDAR pipeline is fully native.
* **Atomic Writes:** `plc_driver_node.py` updated to send both Left and Right speeds in a single physical network packet to prevent micro-rotation jerks.
* **Git Repo Sync:** Submodule `sick_scan_xd` flattened into standard tree directory for easier version control on `AMR-Core`.

*Documentation version 1.3.0 — June 2026* *Ready for SLAM Mapping on actual Polebot physical hardware.*

```

```
