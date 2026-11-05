"""验证真实 launch 展开和配置转发，不打开串口。"""
from pathlib import Path
import tempfile
from unittest.mock import patch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


def test_launch_uses_selected_config():
    share = Path(get_package_share_directory('rcs_imu'))
    with tempfile.TemporaryDirectory() as folder:
        config = Path(folder) / 'imu.yaml'
        config.write_text('rcs_imu:\n  ros__parameters:\n    port: /dev/test_imu\n')
        recorded = []
        def capture(node, context):
            recorded.append((node._Node__node_executable,
                             evaluate_parameters(context, node._Node__parameters)))
            return []
        service = LaunchService(noninteractive=True)
        service.include_launch_description(LaunchDescription([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/rcs_imu.launch.py')),
                                     launch_arguments={'imu_config': str(config)}.items())
        ]))
        with patch.object(Node, 'execute', capture):
            assert service.run() == 0
        assert len(recorded) == 1
        assert recorded[0][0] == 'rcs_imu_node'
        assert Path(recorded[0][1][0]) == config


def test_config_links_to_source():
    config = Path(get_package_share_directory('rcs_imu')) / 'config/rcs_imu.yaml'
    assert config.is_symlink() and config.exists()
    values = yaml.safe_load(config.read_text())['rcs_imu']['ros__parameters']
    assert isinstance(values['port'], str) and values['port']
    assert values['baudrate'] > 0
