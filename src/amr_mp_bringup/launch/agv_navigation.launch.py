#!/usr/bin/env python3
"""
agv_navigation.launch.py — AGV Mode Launch File
=================================================
Stack AGV Polebot:
  - AMCL + map_server  : lokalisasi
  - plc_driver_node    : komunikasi PLC (encoder + motor)
  - kinematics_node    : odometry + cmd_vel → PLC
  - path_follower_node : Pure Pursuit controller
  - path_manager_node  : manajemen jalur tetap
  - ws_bridge_node     : WebSocket bridge ke UI PC

Penggunaan:
  ros2 launch amr_mp_bringup agv_navigation.launch.py \
    map:=/home/eqdev/amr_mp/maps/kantor.yaml
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # -- Argumen --
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/eqdev/amr_mp/maps/kantor.yaml',
        description='Path ke file peta YAML')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan waktu simulasi')

    map_file     = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # -- Path ke config --
    bringup_share   = get_package_share_directory('amr_mp_bringup')
    base_ctrl_share = get_package_share_directory('amr_base_controller')
    hw_share        = get_package_share_directory('amr_hardware_bridge')
    desc_pkg        = get_package_share_directory('amr_mp_description')

    nav2_params_file       = os.path.join(bringup_share,   'config', 'nav2_params_polebot.yaml')
    kinematics_params_file = os.path.join(base_ctrl_share, 'config', 'kinematics_params.yaml')
    agv_params_file        = os.path.join(base_ctrl_share, 'config', 'agv_follower_params.yaml')
    plc_params_file        = os.path.join(hw_share,        'config', 'plc_driver_params.yaml')

    # URDF untuk robot_state_publisher
    xacro_file  = os.path.join(desc_pkg, 'description', 'robot.urdf.xacro')
    robot_desc  = xacro.process_file(xacro_file).toxml()

    # =========================================================================
    # NODES
    # =========================================================================

    # -- Robot State Publisher --
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }])

    # -- Joint State Publisher --
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen')

    # -- Map Server --
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'yaml_filename': map_file, 'use_sim_time': use_sim_time}
        ])

    # -- AMCL --
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time}
        ])

    # -- Lifecycle Manager (map_server + amcl) --
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }])

    # -- SICK Lidar --
    lidar_config_file = os.path.join(bringup_share, 'config', 'sick_tim_5xx_polebot.launch')
    sick_lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_scan_xd',
        output='screen',
        arguments=[lidar_config_file],
        parameters=[{
            'hostname'         : '192.168.3.30',
            'frame_id'         : 'laser_link',
            'tf_base_frame_id' : 'base_link',
        }])

    # -- PLC Driver (Layer 1 — encoder + motor) --
    plc_driver_node = Node(
        package='amr_hardware_bridge',
        executable='plc_driver_node',
        name='plc_driver_node',
        output='screen',
        parameters=[plc_params_file])

    # -- Kinematics Node (cmd_vel → PLC + odometry dari encoder) --
    kinematics_node = Node(
        package='amr_base_controller',
        executable='kinematics_node',
        name='kinematics_node',
        output='screen',
        parameters=[kinematics_params_file])

    # -- Path Follower Node (Pure Pursuit) --
    path_follower_node = Node(
        package='amr_base_controller',
        executable='path_follower_node',
        name='path_follower_node',
        output='screen',
        parameters=[agv_params_file])

    # -- Path Manager Node --
    path_manager_node = Node(
        package='amr_base_controller',
        executable='path_manager_node',
        name='path_manager_node',
        output='screen')

    # -- WebSocket Bridge Node --
    ws_bridge_node = Node(
        package='amr_base_controller',
        executable='ws_bridge_node',
        name='ws_bridge_node',
        output='screen',
        parameters=[agv_params_file])

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        # Hardware
        plc_driver_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        sick_lidar_node,
        kinematics_node,
        # Lokalisasi
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        # AGV stack
        path_follower_node,
        path_manager_node,
        ws_bridge_node,
    ])
