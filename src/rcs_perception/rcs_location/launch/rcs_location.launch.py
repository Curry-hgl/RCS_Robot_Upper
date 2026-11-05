from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start(context):
    share = Path(get_package_share_directory('rcs_location'))
    path = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get('imu_enabled'), bool):
        raise ValueError('rcs_location config requires boolean imu_enabled')

    def resolve(key, default):
        value = config.get(key, '')
        if not isinstance(value, str):
            raise ValueError(f'{key} must be a path string')
        result = Path(value).expanduser() if value else default
        if not result.is_absolute():
            result = path.parent / result
        result = result.resolve()
        if not result.is_file():
            raise ValueError(f'Configuration not found: {result}')
        return str(result)

    actions = []
    if config['imu_enabled']:
        imu_share = Path(get_package_share_directory('rcs_imu'))
        imu_config = resolve('imu_config', imu_share / 'config/rcs_imu.yaml')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(imu_share / 'launch/rcs_imu.launch.py')),
            launch_arguments={'imu_config': imu_config}.items()))
    algorithm_config = resolve('algorithm_config', share / 'config/rcs_location_algorithm.yaml')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/rcs_location_algorithm.launch.py')),
        launch_arguments={'algorithm_config': algorithm_config}.items()))
    return actions


def generate_launch_description():
    config = Path(get_package_share_directory('rcs_location')) / 'config/rcs_location.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)), OpaqueFunction(function=start)])
