"""人工图像验证识别能力；不依赖相机和窗口环境。"""
import unittest
import cv2
import numpy as np
from rcs_vision.detection import detect_blocks, detect_rings
from rcs_vision.qr_code import parse_task_code


class DetectionTests(unittest.TestCase):
    def test_red_block_pixel_geometry(self):
        image = np.full((480, 640, 3), 255, np.uint8)
        cv2.circle(image, (240, 200), 40, (0, 0, 255), -1)
        blocks = detect_blocks(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))
        red = [b for b in blocks if b.color_id == 1]
        self.assertEqual(len(red), 1)
        self.assertLessEqual(abs(red[0].cx - 240), 1)
        self.assertLessEqual(abs(red[0].radius - 40), 2)

    def test_black_ring(self):
        image = np.full((480, 640, 3), 255, np.uint8)
        cv2.circle(image, (320, 240), 70, (0, 0, 0), 15)
        rings = detect_rings(image, cv2.cvtColor(image, cv2.COLOR_BGR2HSV), 40, 100, 500)
        self.assertTrue(any(abs(r.cx - 320) < 5 and abs(r.cy - 240) < 5 for r in rings))

    def test_blank_frame_and_qr_validation(self):
        image = np.full((480, 640, 3), 255, np.uint8)
        self.assertEqual(detect_blocks(cv2.cvtColor(image, cv2.COLOR_BGR2HSV)), [])
        task = parse_task_code('156+123+516+231')
        self.assertTrue(task.valid)
        self.assertEqual(task.batch1_colors, [1, 5, 6])
        self.assertFalse(parse_task_code('789+123+516+231').valid)
