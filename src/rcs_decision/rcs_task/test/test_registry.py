"""验证编号路由、收发限制和编解码拆分后的字节契约。"""
import struct
import unittest
from types import SimpleNamespace as NS

from rcs_msgs.business.rcs_msgs_payload import (
    CHASSIS, VISION, MODE, GRIPPER, BLOCKS, RINGS, TASK_CODE,
    ACTION_COMMAND, ACTION_FEEDBACK, Command, Feedback,
    encode_command, encode_feedback,
)
from rcs_msgs.business.rcs_msgs_registry import Registry, default_registry


class RegistryTests(unittest.TestCase):
    def test_all_outgoing_routes(self):
        registry = default_registry()
        routes = [
            (CHASSIS, NS(mode=1, vx=1.0, vy=2.0, wz=3.0, priority=4),
             bytes((CHASSIS,)) + struct.pack('<BfffB', 1, 1.0, 2.0, 3.0, 4)),
            (VISION, NS(object_id='block', position_x=1.0, position_y=2.0, position_z=3.0),
             bytes((VISION,)) + struct.pack('<Bfff', 1, 1.0, 2.0, 3.0)),
            (BLOCKS, NS(mode=0, objects=[]), b'\x01\x00'),
            (RINGS, NS(mode=1, objects=[]), b'\x02\x00'),
            (TASK_CODE, NS(raw='123+456+123+456'), b'\x03123+456+123+456'),
            (ACTION_COMMAND, Command(7, 1), encode_command(Command(7, 1))),
            (ACTION_FEEDBACK, Feedback(7, 1, 2), encode_feedback(Feedback(7, 1, 2))),
        ]
        for identifier, message, expected in routes:
            with self.subTest(identifier=identifier):
                self.assertEqual(registry.encode(identifier, message), expected)

    def test_incoming_routes_and_receive_only(self):
        registry = default_registry()
        self.assertEqual(registry.decode(b'\x04\x01').mode, 1)
        event = registry.decode(b'\x05\x10\x02\x03')
        self.assertEqual((event.action, event.pick, event.place), (16, 2, 3))
        command, feedback = Command(7, 1), Feedback(7, 1, 2)
        self.assertEqual(registry.decode(encode_command(command)), command)
        self.assertEqual(registry.decode(encode_feedback(feedback)), feedback)
        for identifier in (MODE, GRIPPER):
            with self.assertRaises(ValueError):
                registry.encode(identifier, None)
        for raw in (b'', b'\xff', b'\x04\x02'):
            with self.assertRaises(ValueError):
                registry.decode(raw)

    def test_duplicate_id_is_rejected(self):
        registry = Registry()
        registry.register(MODE, None, lambda raw: raw)
        with self.assertRaises(ValueError):
            registry.register(MODE, bytes)
