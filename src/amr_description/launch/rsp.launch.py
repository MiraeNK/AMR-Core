
import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node





def generate_launch_description():

    pkg_share  = get_package_share_directory('amr_description')

    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    robot_desc = xacro.process_file(xacro_file).toxml()



    use_sim_time = LaunchConfiguration('use_sim_time')



    rsp = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        output='screen',

        parameters=[{

            'robot_description': robot_desc,

            'use_sim_time': use_sim_time,

        }],

    )



    jsp = Node(

        package='joint_state_publisher',

        executable='joint_state_publisher',

        output='screen',

        parameters=[{'use_sim_time': use_sim_time}],

    )



    return LaunchDescription([

        DeclareLaunchArgument(

            'use_sim_time',

            default_value='false',

            description='Use sim time if true',

        ),

        rsp,

        jsp,

    ])

