"""短入口、配置边界及独立任务输入。"""
from pathlib import Path
import tempfile
import unittest

import rclpy
from rcs_task.rcs_task_config import load_system_config, load_task_config
from rcs_task.rcs_task_node import RcsTaskNode
from rcs_task.rcs_task_start import parse_args


class SelectionTests(unittest.TestCase):
    def test_short_cli_selects_complete_task_groups(self):
        self.assertEqual(parse_args([]).tasks, [])
        self.assertEqual(parse_args(['upper', 'vision']).tasks, ('upper', 'vision'))
        self.assertEqual(parse_args(['vision', '--config', '/tmp/tasks.yaml']).config, '/tmp/tasks.yaml')

    def test_system_config_only_selects_tasks_and_profile_paths(self):
        self.assertEqual(load_system_config()[0], ('vision',))
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'config.yaml'
            path.write_text('tasks: [upper, chassis]\ntask_configs:\n  upper: custom.yaml\n')
            tasks, profiles = load_system_config(path)
            self.assertEqual(tasks, ('upper', 'chassis'))
            self.assertEqual(profiles['upper'], str(Path(d) / 'custom.yaml'))
            self.assertEqual(load_system_config(path, tasks=['vision'])[0], ('vision',))

    def test_each_node_subscribes_only_to_its_task_inputs(self):
        expected = {
            'upper': {'/rcs_task/gripper_event'},
            'chassis': {'/rcs_task/chassis/command'},
            'vision': {'/rcs_vision/qr_candidate', '/rcs_task/mode_command',
                       '/rcs_vision/detections', '/rcs_task/gripper_event'},
        }
        rclpy.init(args=[])
        try:
            for name, topics in expected.items():
                node = RcsTaskNode(tasks=[name], config={'tasks': [name], 'qr_enabled': False})
                try:
                    self.assertEqual({sub.topic_name for sub in node.subs}, topics)
                    node._tick()
                    self.assertEqual(list(node.task.tasks), [name])
                finally:
                    node.task.close()
                    node.destroy_node()
        finally:
            rclpy.shutdown()

    def test_vision_profile_resolves_its_own_algorithm_file(self):
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d)
            (folder / 'vision.yaml').write_text('width: 800\ninitial_mode: 1\n')
            path = folder / 'task.yaml'
            path.write_text('vision_config: vision.yaml\nserial_enabled: false\n')
            config = load_task_config('vision', path)
            self.assertEqual(config['width'], 800)
            self.assertEqual(config['initial_mode'], 1)
            self.assertEqual(config['vision_config'], str(folder / 'vision.yaml'))
            self.assertFalse(config['serial_enabled'])
            self.assertEqual(config['tasks'], ['vision'])
