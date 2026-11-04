"""无硬件RC 国赛闭环：人工相机→真实算法→任务→ROS→RDLC→伪串口。

模拟下位机发送黑环模式与抓放统计，验证回程消息也进入视觉与任务。
通过 ros2 run rcs_task test serial_link 运行；不打开窗口。
"""
import os
import pty
import select
import threading
import time
from unittest.mock import patch

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rcs_serial.rcs_serial_bridge import SerialBridge
from rcs_serial.rcs_serial_config import SerialConfig
from rcs_vision.rcs_vision_node import RcsVision
from rcs_task.rcs_task_node import RcsTaskNode
from rcs_ptl import FrameCodec


class SyntheticCamera:
    """只替换硬件采集；后面的 HSV/轮廓/Hough 算法全部真实运行。"""
    ring = False
    def __init__(self, *args): pass
    def read(self):
        image = np.full((480, 640, 3), 255, np.uint8)
        if self.ring:
            cv2.circle(image, (320, 240), 70, (0, 0, 0), 15)
        else:
            cv2.circle(image, (240, 200), 40, (0, 0, 255), -1)
        return image
    def close(self): pass


class SyntheticQr:
    def __init__(self, *args): self.frames = 0
    def decode(self, frame):
        self.frames += 1
        return ['156+123+516+231'] if self.frames > 15 else []


def test_serial_link():
    SyntheticCamera.ring = False
    master, slave = pty.openpty()
    stop = threading.Event()
    seen, errors = [], []
    def lower_board():
        codec = FrameCodec(escape_enabled=True)
        switched = False
        try:
            while not stop.is_set():
                if not select.select([master], [], [], 0.05)[0]:
                    continue
                for frame in codec.feed(os.read(master, 1024)):
                    seen.append(frame.payload)
                    if frame.payload[:1] == b'\x03' and not switched:
                        switched = True
                        SyntheticCamera.ring = True
                        # 清零后加一次抓取、一次放置，事件不能合并丢序。
                        for raw in (b'\x04\x01', b'\x05\x05', b'\x05\x01', b'\x05\x03'):
                            os.write(master, codec.encode(2, 1, raw))
        except Exception as exc:
            errors.append(exc)
    rclpy.init()
    bridge = SerialBridge(SerialConfig(os.ttyname(slave), escape_enabled=True))
    bridge.open()
    task = RcsTaskNode(tasks=['upper', 'vision'])
    with patch('rcs_vision.rcs_vision_node.Camera', SyntheticCamera), patch('rcs_vision.rcs_vision_node.QrDetector', SyntheticQr):
        vision = RcsVision()
    executor = SingleThreadedExecutor()
    for node in (bridge, task, vision): executor.add_node(node)
    thread = threading.Thread(target=lower_board)
    thread.start()
    try:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if (any(p[:2] == b'\x01\x01' for p in seen)
                    and any(p[:2] == b'\x02\x01' for p in seen)
                    and task.task.tasks['upper'].pick_count == task.task.tasks['upper'].place_count == 1):
                break
        assert not errors, errors
        assert any(p[:2] == b'\x01\x01' for p in seen), 'No block output'
        assert b'\x03156+123+516+231' in seen, 'No confirmed QR output'
        assert any(p[:2] == b'\x02\x01' for p in seen), 'No ring output'
        assert task.task.tasks['vision'].mode == vision.mode == 1
        assert task.task.tasks['upper'].pick_count == task.task.tasks['upper'].place_count == 1
        print('PASS: synthetic image -> algorithms -> FSM -> ROS -> RDLC -> PTY; mode/count feedback returned')
    finally:
        stop.set(); thread.join()
        vision.close(); bridge.close()
        for node in (vision, task, bridge): node.destroy_node()
        executor.shutdown(); rclpy.shutdown()
        os.close(master); os.close(slave)


if __name__ == '__main__':
    test_serial_link()
