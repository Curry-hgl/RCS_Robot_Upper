"""独立 IMU 启动，可由任务 launch 包含。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('rcs_imu')) / 'config/rcs_imu.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('imu_config', default_value=str(config)),
        Node(package='rcs_imu', executable='rcs_imu_node', name='rcs_imu',
             output='screen',
             parameters=[LaunchConfiguration('imu_config')]),
    ])
