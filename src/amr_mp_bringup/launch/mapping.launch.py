import os

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                             RegisterEventHandler, ExecuteProcess)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node





def generate_launch_description():

    

    desc_pkg = get_package_share_directory('amr_mp_description')

    hw_pkg   = get_package_share_directory('amr_hardware_bridge')

    kin_pkg  = get_package_share_directory('amr_base_controller')

    tel_pkg  = get_package_share_directory('amr_mp_teleop')

    slam_pkg = get_package_share_directory('slam_toolbox')

    this_pkg = get_package_share_directory('amr_mp_bringup')



    xacro_file = os.path.join(desc_pkg, 'description', 'robot.urdf.xacro')

    robot_desc = xacro.process_file(xacro_file).toxml()

    rviz_config_path = os.path.join(this_pkg, 'config', 'mapping.rviz')

    maps_dir = os.path.expanduser('~/maps')

    os.makedirs(maps_dir, exist_ok=True)

    map_path = os.path.join(maps_dir, 'gudang')


    # 1. Robot State Publisher

    rsp = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        name='robot_state_publisher',

        output='screen',

        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],

    )



    # 2. PLC Driver Node (Layer 1)

    plc_driver = Node(

        package='amr_hardware_bridge',

        executable='plc_driver_node',

        name='plc_driver_node',

        output='screen',

        parameters=[os.path.join(hw_pkg, 'config', 'plc_driver_params.yaml')],

    )



    # 3. Kinematics Node (Layer 2B — proven working)

    # Menggantikan diff_drive_controller sementara

    # Publish /odom + TF odom->base_footprint yang dibutuhkan slam_toolbox

    kinematics = Node(

        package='amr_base_controller',

        executable='kinematics_node',

        name='kinematics_node',

        output='screen',

        parameters=[os.path.join(kin_pkg, 'config', 'kinematics_params.yaml')],

    )



    # 4. Joint State Publisher — publish wheel joint states untuk RSP

    jsp = Node(

        package='joint_state_publisher',

        executable='joint_state_publisher',

        name='joint_state_publisher',

        output='screen',

    )



    # 5. SLAM Toolbox — delay 3s tunggu TF odom tersedia

    slam = TimerAction(

        period=3.0,

        actions=[IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),

            launch_arguments={

                'use_sim_time': 'false',

                'slam_params_file': os.path.join(

                    this_pkg, 'config', 'slam_toolbox_params.yaml'),

            }.items(),

        )],

    )



    # 6. Manual Control Node

    teleop = Node(

        package='amr_mp_teleop',

        executable='manual_control_node',

        name='manual_control_node',

        output='screen',

        parameters=[os.path.join(tel_pkg, 'config', 'teleop_params.yaml')],

        prefix='xterm -e',

    )
    
    # 7. auto open rviz2
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        
        prefix=['xterm -e'] 
    )



    # 8. Auto-save map saat shutdown (Ctrl+C)

    save_map = RegisterEventHandler(

        OnShutdown(

            on_shutdown=[

                ExecuteProcess(

                    cmd=[

                        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',

                        '-f', map_path,

                        '--ros-args', '-p', 'save_map_timeout:=5.0'

                    ],

                    output='screen',

                    name='map_saver',

                )

            ]

        )

    )



    return LaunchDescription([

        rsp,

        plc_driver,

        kinematics,

        jsp,

        slam,

        teleop,
        
        rviz_node,
        save_map,
    ])

