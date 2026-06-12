import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    desc_pkg = get_package_share_directory('amr_mp_description')
    hw_pkg   = get_package_share_directory('amr_hardware_bridge')
    kin_pkg  = get_package_share_directory('amr_base_controller')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    this_pkg = get_package_share_directory('amr_mp_bringup')

    map_file         = LaunchConfiguration('map')
    
    # --- PERBAIKAN DI SINI ---
    # Mengambil konfigurasi RViz yang super lengkap langsung dari bawaan Nav2
    rviz_config_path = os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')
    # -------------------------
    
    xacro_file       = os.path.join(desc_pkg, 'description', 'robot.urdf.xacro')
    robot_desc       = xacro.process_file(xacro_file).toxml()
    lidar_config_file = os.path.join(this_pkg, 'config', 'sick_tim_5xx_polebot.launch')

    # 0. SICK LiDAR
    sick_lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='screen',
        arguments=[lidar_config_file],
    )

    # 1. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
    )

    # 2. PLC Driver
    plc_driver = Node(
        package='amr_hardware_bridge',
        executable='plc_driver_node',
        name='plc_driver_node',
        output='screen',
        parameters=[os.path.join(hw_pkg, 'config', 'plc_driver_params.yaml')],
    )

    # 3. Kinematics
    kinematics = Node(
        package='amr_base_controller',
        executable='kinematics_node',
        name='kinematics_node',
        output='screen',
        parameters=[os.path.join(kin_pkg, 'config', 'kinematics_params.yaml')],
    )

    # 4. Joint State Publisher
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # 5. Nav2 Navigation (Planner + Controller) - Dijalankan pertama kali
    nav2_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file' : os.path.join(this_pkg, 'config', 'nav2_params.yaml'),
            # --- BARIS SAKTI DITAMBAHKAN DI SINI ---
            # Ini akan memaksa Nav2 memakai logika "Kunci Jalur & Tanpa Spin" buatan kita!
            'default_bt_xml_filename': os.path.join(this_pkg, 'config', 'amr_nav_tree.xml'),
            # ---------------------------------------
        }.items(),
    )

    # 6. Nav2 Localization (AMCL + Map Server) - Delay 4 detik
    # Menunda Map Server agar Global Costmap standby terlebih dahulu
    nav2_loc = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, 'launch', 'localization_launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'map'         : map_file,
                'params_file' : os.path.join(this_pkg, 'config', 'nav2_params.yaml'),
            }.items(),
        )]
    )

    # 7. RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/maps/gudang.yaml'),
            description='Path ke file peta .yaml',
        ),
        sick_lidar_node,
        rsp,
        plc_driver,
        kinematics,
        jsp,
        nav2_nav,
        nav2_loc,
        rviz2_node,
    ])
