import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch.actions import GroupAction
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    main_dir = get_package_share_directory('449_ADventure')

    # ros2 launch nav2_bringup navigation_launch.py
    nav2 = GroupAction(
            actions=[
                SetRemap(src='/cmd_vel', dst='/commands/velocity'),


            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
                launch_arguments = {
                }.items()
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
                launch_arguments = {
                    'slam_params_file': os.path.join(main_dir, 'config', 'nav_params.yaml')
                }.items()
            )
    ]
        )


    ld = LaunchDescription()
#    ld.add_action(start_nav2)
#    ld.add_action(start_slam_toolbox)

    ld.add_action(nav2)
    return ld

