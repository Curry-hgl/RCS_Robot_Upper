"""底盘任务及可选 IMU。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rcs_task.rcs_task_launch import task_launch, task_node


def build_nodes(config, path):
    nodes = [task_node('chassis', config, path)]
    if config['imu_enabled']:
        share = Path(get_package_share_directory('rcs_imu'))
        imu_config = config['imu_config'] or str(share / 'config/rcs_imu.yaml')
        nodes.append(GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / 'launch/rcs_imu.launch.py')),
                launch_arguments={'imu_config': imu_config}.items())
        ]))
    return nodes


def generate_launch_description():
    return task_launch('chassis', build_nodes)
