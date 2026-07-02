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

    # FIX 1: Ganti nama peta ke 'kantor' dan gunakan ~/amr_mp/maps/
    maps_dir = os.path.expanduser('~/amr_mp/maps')
    os.makedirs(maps_dir, exist_ok=True)
    map_path = os.path.join(maps_dir, 'kantor')

    lidar_config_file = os.path.join(this_pkg, 'config', 'sick_tim_5xx_polebot.launch')

    sick_lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=[lidar_config_file],
        parameters=[{
            'hostname'         : '192.168.3.30',
            'frame_id'         : 'laser_link',
            'tf_base_frame_id' : 'base_link',
        }]
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
    )

    plc_driver = Node(
        package='amr_hardware_bridge',
        executable='plc_driver_node',
        name='plc_driver_node',
        output='screen',
        parameters=[os.path.join(hw_pkg, 'config', 'plc_driver_params.yaml')],
    )

    kinematics = Node(
        package='amr_base_controller',
        executable='kinematics_node',
        name='kinematics_node',
        output='screen',
        parameters=[os.path.join(kin_pkg, 'config', 'kinematics_params.yaml')],
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    slam = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(
                    this_pkg, 'config', 'slam_toolbox_params.yaml'),
            }.items(),
        )],
    )

    teleop = Node(
        package='amr_mp_teleop',
        executable='manual_control_node',
        name='manual_control_node',
        output='screen',
        parameters=[os.path.join(tel_pkg, 'config', 'teleop_params.yaml')],
        prefix='xterm -e',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        prefix=['xterm -e']
    )

    # FIX 2: Dua metode save — map_saver_cli (pgm+yaml) + slam serialize (posegraph)
    # map_saver_cli tanpa timeout parameter (Foxy tidak support float di sini)
    # slam serialize sebagai backup jika map_saver gagal
    save_map = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                # Metode 1: standard pgm + yaml (untuk Nav2/AMCL)
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                        '-f', map_path,
                    ],
                    output='screen',
                    name='map_saver_pgm',
                ),
                # Metode 2: slam_toolbox serialize (backup posegraph)
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/slam_toolbox/serialize_map',
                        'slam_toolbox/srv/SerializePoseGraph',
                        '{filename: \'' + map_path + '\'}',
                    ],
                    output='screen',
                    name='map_saver_posegraph',
                ),
            ]
        )
    )

    return LaunchDescription([
        sick_lidar_node,
        rsp,
        plc_driver,
        kinematics,
        jsp,
        slam,
        teleop,
        rviz_node,
        save_map,
    ])
