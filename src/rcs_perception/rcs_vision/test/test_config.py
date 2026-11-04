"""视觉独立配置、ROS 覆盖和相机/预览选择；不接触真实设备。"""
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
import rclpy

from rcs_vision.rcs_vision_config import load_vision_config
from rcs_vision.rcs_vision_node import RcsVision, parse_args


class Camera:
    def __init__(self, source, width, height):
        self.source = source
        self.closed = False
    def read(self):
        return np.full((120, 160, 3), 255, dtype=np.uint8)
    def close(self): self.closed = True


class Qr:
    def __init__(self, backend, model_dir): pass
    def decode(self, frame): return ['156+123+516+231']


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.patches = [patch('rcs_vision.rcs_vision_node.Camera', side_effect=Camera),
                        patch('rcs_vision.rcs_vision_node.QrDetector', side_effect=Qr),
                        patch('rcs_vision.rcs_vision_node.cv2.imshow'),
                        patch('rcs_vision.rcs_vision_node.cv2.waitKey'),
                        patch('rcs_vision.rcs_vision_node.cv2.destroyWindow'),
                        patch('rcs_vision.rcs_vision_node.cv2.destroyAllWindows')]
        self.camera, self.qr, self.show, self.wait, self.destroy, _ = [p.start() for p in self.patches]
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            node.close()
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for p in reversed(self.patches): p.stop()

    def node(self, config=None):
        node = RcsVision(config=config)
        self.nodes.append(node)
        return node

    def test_standalone_uses_installed_yaml(self):
        config = load_vision_config()
        rclpy.init(args=[])
        node = self.node()
        self.camera.assert_any_call(config['block_camera'], config['width'], config['height'])
        self.assertEqual(node.mode, config['initial_mode'])
        self.assertEqual(node.block_args['min_radius'], config['block_min_radius'])

    def test_custom_yaml_and_ros_override(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'vision.yaml'
            path.write_text("block_camera: custom\\nwidth: 800\\nheight: 600\\ninitial_mode: 1\\n"
                            "qr_enabled: false\\nring_min_radius: 35\\n".replace('\\n', '\n'))
            rclpy.init(args=['--ros-args', '-p', 'width:=320'])
            node = self.node(load_vision_config(path))
            self.camera.assert_called_once_with('custom', 320, 600)
            self.qr.assert_not_called()
            self.assertEqual(node.mode, 1)
            self.assertEqual(node.ring_args['min_radius'], 35)

    def test_shared_camera_only_opens_once(self):
        rclpy.init(args=[])
        node = self.node(dict(block_camera='0', qr_camera='0', qr_enabled=True))
        self.assertEqual(self.camera.call_count, 1)
        self.assertIsNone(node.qr_camera)

    def test_qr_preview_closes_on_confirmation_without_stopping_main_camera(self):
        rclpy.init(args=[])
        node = self.node(dict(block_camera='main', qr_camera='qr', qr_enabled=True,
                              preview=False, qr_preview=True))
        qr_camera = node.qr_camera
        node._tick()
        self.assertEqual(self.show.call_args.args[0], 'RCS QR')
        self.assertEqual(self.show.call_count, 1)
        self.wait.assert_called_once()
        node._status(NS(task_code='156+123+516+231'))
        self.destroy.assert_called_once_with('RCS QR')
        self.assertTrue(qr_camera.closed)
        self.assertFalse(node.camera.closed)
        node._tick()
        self.assertEqual(self.show.call_count, 1)

    def test_qr_preview_runs_when_main_frame_is_missing(self):
        rclpy.init(args=[])
        node = self.node(dict(block_camera='main', qr_camera='qr', qr_enabled=True,
                              preview=False, qr_preview=True))
        node.camera.read = lambda: None
        node._tick()
        self.show.assert_called_once()
        self.wait.assert_called_once()

    def test_config_cli_keeps_ros_arguments_separate(self):
        options = parse_args(['rcs_vision', '--config', '/tmp/custom.yaml',
                              '--ros-args', '-p', 'initial_mode:=1'])
        self.assertEqual(options.config, '/tmp/custom.yaml')
