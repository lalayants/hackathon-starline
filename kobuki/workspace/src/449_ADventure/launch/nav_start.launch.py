import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    main_dir = get_package_share_directory('449_ADventure')

    # ros2 launch nav2_bringup navigation_launch.py
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments = {
        }.items()
    )

    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments = {
            'slam_params_file': os.path.join(main_dir, 'config', 'nav_params.yaml')
        }.items()
    )



    ld = LaunchDescription()
    ld.add_action(start_nav2)
    ld.add_action(start_slam_toolbox)

    return ld

