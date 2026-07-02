#!/usr/bin/env python3
"""
agv_virtual_line.launch.py — AGV Virtual Magnetic Line Mode
"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    hw_share    = get_package_share_directory('amr_hardware_bridge')
    base_share  = get_package_share_directory('amr_base_controller')
    bringup_share = get_package_share_directory('amr_mp_bringup')
    desc_pkg    = get_package_share_directory('amr_mp_description')

    plc_params      = os.path.join(hw_share,      'config', 'plc_driver_params.yaml')
    follower_params = os.path.join(hw_share,      'config', 'agv_line_follower_params.yaml')
    kine_params     = os.path.join(base_share,    'config', 'kinematics_params.yaml')
    lidar_config    = os.path.join(bringup_share, 'config', 'sick_tim_5xx_polebot.launch')

    xacro_file  = os.path.join(desc_pkg, 'description', 'robot.urdf.xacro')
    robot_desc  = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_scan_xd',
            output='screen',
            arguments=[lidar_config],
            parameters=[{
                'hostname'         : '192.168.3.30',
                'frame_id'         : 'laser_link',
                'tf_base_frame_id' : 'base_link',
            }],
        ),
        Node(
            package='amr_hardware_bridge',
            executable='plc_driver_node',
            name='plc_driver_node',
            output='screen',
            parameters=[plc_params],
        ),
        Node(
            package='amr_base_controller',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[kine_params],
        ),
        Node(
            package='amr_hardware_bridge',
            executable='agv_line_follower_node',
            name='agv_line_follower_node',
            output='screen',
            parameters=[follower_params],
        ),
        Node(
            package='amr_base_controller',
            executable='ws_bridge_node',
            name='ws_bridge_node',
            output='screen',
        ),
    ])
