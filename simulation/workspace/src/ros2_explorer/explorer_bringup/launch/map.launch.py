#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    cartographer_launch_file_dir = os.path.join(get_package_share_directory('explorer_cartographer'), 'launch')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('explorer_navigation2'), 'launch')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([cartographer_launch_file_dir, '/cartographer.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/nav.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='explorer_wanderer',
            executable='wanderer_server',
            name='wanderer_server',
            output='screen',
        ),
    ])