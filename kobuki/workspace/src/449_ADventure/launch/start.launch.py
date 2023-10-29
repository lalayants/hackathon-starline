import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    kobuki_pkg_dir  = get_package_share_directory('kobuki_node')
    lidar_pkg_dir   = get_package_share_directory('rplidar_ros')
    astra_pkg_dir   = get_package_share_directory('astra_camera')



    start_kobuki = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(kobuki_pkg_dir, 'launch', 'kobuki.launch.py')),
        launch_arguments = {}
    )

    start_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg_dir, 'launch', 'rplidar.launch.py')),
        launch_arguments = {}
    )

    start_astra = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(astra_pkg_dir, 'launch', 'dabai_dw.launch.py')),
        launch_arguments = {}
    )

    static_transformer_base_camera = Node(
	package='tf2_ros',
	executable='static_transform_publisher',
	arguments= ["-0.105", "0", "0.24", "0", "0", "0", "base_link", "camera_link"]
	)

    static_transformer_base_laser = Node(
	package='tf2_ros',
	executable='static_transform_publisher',
	arguments= ["0", "0", "0.36", "0", "0", "0", "base_link", "laser"]
	)

    static_transformer_base_footprint = Node(
	package='tf2_ros',
	executable='static_transform_publisher',
	arguments= ["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"]
	)

    ld = LaunchDescription()
    ld.add_action(start_kobuki)
    ld.add_action(start_lidar)
    ld.add_action(start_astra)
    ld.add_action(static_transformer_base_camera)
    ld.add_action(static_transformer_base_laser)
    ld.add_action(static_transformer_base_footprint)

    return ld

