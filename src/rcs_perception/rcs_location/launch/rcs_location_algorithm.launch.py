from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('rcs_location')) / 'config/rcs_location_algorithm.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('algorithm_config', default_value=str(config)),
        Node(package='rcs_location', executable='rcs_location_algorithm_node',
             name='rcs_location_algorithm', output='screen',
             parameters=[LaunchConfiguration('algorithm_config')]),
    ])
