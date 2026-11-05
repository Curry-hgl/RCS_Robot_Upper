"""执行真实子 launch 展开，拦截 Node 启动以验证组合而不访问硬件。"""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

from rcs_task.rcs_task_launch import system_launch


class LaunchTests(unittest.TestCase):
    def expand(self, description):
        records = []
        def capture(node, context):
            records.append(dict(
                package=node._Node__package, executable=node._Node__node_executable,
                name=node._Node__node_name,
                parameters=evaluate_parameters(context, node._Node__parameters),
                remappings=[(perform_substitutions(context, a), perform_substitutions(context, b))
                            for a, b in node._Node__remappings],
            ))
            return []
        service = LaunchService(noninteractive=True)
        service.include_launch_description(description)
        with patch.object(Node, 'execute', capture):
            result = service.run()
        return result, records

    def profiles(self, folder, tasks, **overrides):
        paths = {}
        for name in tasks:
            path = folder / f'{name}.yaml'
            path.write_text(yaml.safe_dump(overrides.get(name, {})))
            paths[name] = str(path)
        system = folder / 'system.yaml'
        system.write_text(yaml.safe_dump({'tasks': list(tasks), 'task_configs': paths}))
        return system, paths

    def test_standalone_vision_launch_contains_whole_stack(self):
        launch = Path(get_package_share_directory('rcs_task')) / 'launch/rcs_task_vision.launch.py'
        result, nodes = self.expand(LaunchDescription([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)))
        ]))
        self.assertEqual(result, 0)
        self.assertEqual([n['executable'] for n in nodes],
                         ['serial_bridge', 'rcs_task_node', 'rcs_vision', 'rcs_debug_monitor'])

    def test_combination_includes_children_and_reuses_serial(self):
        for tasks in [('upper',), ('chassis',), ('upper', 'vision'), ('upper', 'chassis', 'vision')]:
            with self.subTest(tasks=tasks), tempfile.TemporaryDirectory() as folder:
                profiles = {name: {'serial_enabled': True, 'port': '/dev/test_bus',
                                   'baudrate': 115200, 'escape_enabled': False}
                            for name in tasks}
                system, _ = self.profiles(Path(folder), tasks, **profiles)
                result, nodes = self.expand(system_launch(config=system))
                self.assertEqual(result, 0)
                self.assertEqual(sum(n['executable'] == 'serial_bridge' for n in nodes), 1)
                task_nodes = [n for n in nodes if n['executable'] == 'rcs_task_node']
                self.assertEqual([n['name'] for n in task_nodes], [f'rcs_task_{t}' for t in tasks])
                self.assertEqual(len({n['name'] for n in nodes}), len(nodes))
                for name, node in zip(tasks, task_nodes):
                    self.assertEqual(node['parameters'][0]['tasks'], name)
                    self.assertIn(('/rcs_task/status', f'/rcs_task/{name}/status'), node['remappings'])
                for node in nodes:
                    if node['executable'] in ('rcs_vision', 'rcs_debug_monitor'):
                        self.assertIn(('/rcs_task/status', '/rcs_task/vision/status'), node['remappings'])

    def test_task_profiles_control_flags_and_do_not_leak_across_children(self):
        with tempfile.TemporaryDirectory() as d:
            system, _ = self.profiles(Path(d), ('upper', 'vision'),
                                      upper={'serial_enabled': False, 'tick_period': 0.01},
                                      vision={'serial_enabled': False, 'monitor': False, 'tick_period': 0.2})
            result, nodes = self.expand(system_launch(config=system))
            self.assertEqual(result, 0)
            self.assertNotIn('serial_bridge', [n['executable'] for n in nodes])
            self.assertNotIn('rcs_debug_monitor', [n['executable'] for n in nodes])
            periods = {n['name']: n['parameters'][0]['tick_period'] for n in nodes
                       if n['executable'] == 'rcs_task_node'}
            self.assertEqual(periods, {'rcs_task_upper': 0.01, 'rcs_task_vision': 0.2})

    def test_any_enabled_task_can_request_shared_serial(self):
        with tempfile.TemporaryDirectory() as d:
            system, _ = self.profiles(Path(d), ('upper', 'vision'),
                                      upper={'serial_enabled': False},
                                      vision={'serial_enabled': True, 'monitor': False})
            result, nodes = self.expand(system_launch(config=system))
            self.assertEqual(result, 0)
            self.assertEqual(sum(n['executable'] == 'serial_bridge' for n in nodes), 1)

    def test_conflicting_serial_profiles_fail_before_any_process(self):
        with tempfile.TemporaryDirectory() as d:
            system, _ = self.profiles(Path(d), ('upper', 'vision'),
                                      upper={'baudrate': 9600}, vision={'baudrate': 115200})
            result, nodes = self.expand(system_launch(config=system))
            self.assertNotEqual(result, 0)
            self.assertEqual(nodes, [])

    def test_direct_child_obeys_its_configuration_file(self):
        with tempfile.TemporaryDirectory() as d:
            _, paths = self.profiles(Path(d), ('vision',),
                                      vision={'serial_enabled': False, 'monitor': False, 'qr_confirm_frames': 5})
            launch = Path(get_package_share_directory('rcs_task')) / 'launch/rcs_task_vision.launch.py'
            result, nodes = self.expand(LaunchDescription([
                IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)),
                                         launch_arguments={'config': paths['vision']}.items())
            ]))
            self.assertEqual(result, 0)
            self.assertEqual([n['executable'] for n in nodes], ['rcs_task_node', 'rcs_vision'])
            self.assertEqual(nodes[0]['parameters'][0]['qr_confirm_frames'], 5)

    def test_short_launcher_drives_launch_service(self):
        from rcs_task.rcs_task_start import main
        with patch('rcs_task.rcs_task_start.LaunchService') as launch:
            launch.return_value.run.return_value = 7
            self.assertEqual(main(['upper', 'vision']), 7)
            launch.return_value.include_launch_description.assert_called_once()
            launch.return_value.run.assert_called_once()

    def test_chassis_can_include_imu_once(self):
        with tempfile.TemporaryDirectory() as d:
            system, _ = self.profiles(Path(d), ('upper', 'chassis'),
                                      upper={'serial_enabled': False},
                                      chassis={'serial_enabled': False, 'imu_enabled': True})
            result, nodes = self.expand(system_launch(config=system))
            self.assertEqual(result, 0)
            self.assertEqual(sum(n['executable'] == 'rcs_imu_node' for n in nodes), 1)

    def test_imu_serial_conflict_fails_before_any_process(self):
        with tempfile.TemporaryDirectory() as d:
            imu = Path(d) / 'imu.yaml'
            imu.write_text(yaml.safe_dump({'rcs_imu': {'ros__parameters': {'port': '/dev/test_bus'}}}))
            system, _ = self.profiles(Path(d), ('upper', 'chassis'),
                                      upper={'port': '/dev/test_bus'},
                                      chassis={'serial_enabled': False, 'imu_enabled': True,
                                               'imu_config': 'imu.yaml'})
            result, nodes = self.expand(system_launch(config=system))
            self.assertNotEqual(result, 0)
            self.assertEqual(nodes, [])
