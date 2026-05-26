import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node





def generate_launch_description():

    desc_pkg = get_package_share_directory('amr_mp_description')

    hw_pkg   = get_package_share_directory('amr_hardware_bridge')

    kin_pkg  = get_package_share_directory('amr_base_controller')

    nav2_pkg = get_package_share_directory('nav2_bringup')

    slam_pkg = get_package_share_directory('slam_toolbox')

    this_pkg = get_package_share_directory('amr_mp_bringup')



    map_file = LaunchConfiguration('map')



    xacro_file = os.path.join(desc_pkg, 'description', 'robot.urdf.xacro')

    robot_desc = xacro.process_file(xacro_file).toxml()



    # 1. Robot State Publisher

    rsp = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        name='robot_state_publisher',

        output='screen',

        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],

    )



    # 2. PLC Driver Node

    plc_driver = Node(

        package='amr_hardware_bridge',

        executable='plc_driver_node',

        name='plc_driver_node',

        output='screen',

        parameters=[os.path.join(hw_pkg, 'config', 'plc_driver_params.yaml')],

    )



    # 3. Kinematics Node (Layer 2B — proven working)

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



    # 5. SLAM Toolbox localization mode

    slam_loc = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(slam_pkg, 'launch', 'localization_launch.py')),

        launch_arguments={

            'use_sim_time': 'false',

            'slam_params_file': os.path.join(

                this_pkg, 'config',

                'slam_toolbox_localization_params.yaml'),

            'map': map_file,

        }.items(),

    )



    # 6. Nav2

    nav2 = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),

        launch_arguments={

            'use_sim_time': 'false',

            'params_file': os.path.join(

                this_pkg, 'config', 'nav2_params.yaml'),

        }.items(),

    )



    return LaunchDescription([

        DeclareLaunchArgument(

            'map',

            default_value=os.path.expanduser('~/maps/gudang.yaml'),

            description='Path ke file peta .yaml',

        ),

        rsp,

        plc_driver,

        kinematics,

        jsp,

        slam_loc,

        nav2,

    ])

