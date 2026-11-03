"""工创任务与原字节协议回归：编号、帧结构、阶段及顺序事件。"""
import struct
import unittest
from types import SimpleNamespace as NS
from rcs_msgs.business.gcs import encode_detections, encode_task_code, decode_mode, decode_gripper
from rcs_msgs.business import CHASSIS, VISION, default_registry
from rcs_task.gcs import GcsTask, Phase


class Actions:
    def __init__(self):
        self.codes, self.detections, self.modes = [], [], []
    def set_mode(self, mode): self.modes.append(mode)
    def send_code(self, code): self.codes.append(code)
    def send_detections(self, msg): self.detections.append(msg)


class GcsTests(unittest.TestCase):
    def test_original_wire_format_and_id_conflicts(self):
        obj = NS(color_id=1, cx=240.0, cy=200.0, radius=40.0)
        for mode in (0, 1):
            msg = NS(mode=mode, objects=[obj] * 4)
            self.assertEqual(encode_detections(msg), bytes((mode + 1, 3)) + struct.pack('<Bfff', 1, 240, 200, 40) * 3)
        self.assertEqual(encode_task_code(NS(raw='156+123+516+231')), b'\x03156+123+516+231')
        self.assertEqual(decode_mode(b'\x04\x01').mode, 1)
        self.assertEqual(decode_gripper(b'\x05\x10\x02\x03').place, 3)
        self.assertEqual(CHASSIS, 0x20)
        self.assertEqual(VISION, 0x21)
        self.assertEqual(default_registry().decode(b'\x04\x00').mode, 0)
        with self.assertRaises(ValueError): decode_gripper(b'\x05\x10')

    def test_confirmation_broadcast_and_tracking(self):
        now = [0.0]
        actions = Actions()
        task = GcsTask(actions, lambda: now[0], broadcast_duration=1.0)
        code = '156+123+516+231'
        task.on_qr(code); task.on_qr(''); task.on_qr(code); task.on_qr(code)
        self.assertEqual(actions.codes, [])
        task.on_qr(code)
        self.assertEqual(actions.codes, [code])
        task.tick(); task.tick()
        self.assertEqual(task.machine.state, Phase.BROADCAST)
        now[0] = 2
        task.tick(); task.tick()
        self.assertEqual(task.machine.state, Phase.TRACK)
        count = len(actions.codes)
        now[0] = 3
        task.tick()
        self.assertEqual(len(actions.codes), count)
        task.on_mode(1)
        task.on_detections(NS(mode=0, objects=[]))
        self.assertIsNone(task.latest)
        task.on_detections(NS(mode=1, objects=[]))
        now[0] = 3.5
        task.tick()
        self.assertEqual(actions.detections[-1].mode, 1)

    def test_gripper_event_order_and_stale_frames(self):
        now = [0]
        actions = Actions()
        task = GcsTask(actions, lambda: now[0], qr_enabled=False)
        for raw in (b'\x05\x01', b'\x05\x05', b'\x05\x01', b'\x05\x03'):
            task.on_gripper(decode_gripper(raw))
        self.assertEqual((task.pick_count, task.place_count), (1, 1))
        task.on_detections(NS(mode=0, objects=[]))
        now[0] = 4
        task.tick()
        self.assertEqual(actions.detections, [])
