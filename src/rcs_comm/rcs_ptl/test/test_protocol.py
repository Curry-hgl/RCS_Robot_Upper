"""替换默认协议模块后，上层 frame 接口无需更改。"""
import struct
import unittest
from unittest.mock import patch

from rcs_ptl import Frame, FrameCodec, FrameDecodeError


class AlternateProtocol:
    """测试用不同线格式：A5 + 16 位源/目的地址 + 16 位长度 + 原始载荷。"""

    def __init__(self, *, marker=0xA5):
        self.marker = marker
        self.reset()

    def reset(self):
        self.buffer = bytearray()

    def encode(self, src, dst, payload):
        return bytes((self.marker,)) + struct.pack('<HHH', src, dst, len(payload)) + payload

    def feed(self, data, *, strict=False):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            if self.buffer[0] != self.marker:
                self.buffer.pop(0)
                if strict:
                    raise FrameDecodeError('Invalid alternate header')
                continue
            if len(self.buffer) < 7:
                break
            src, dst, size = struct.unpack('<HHH', self.buffer[1:7])
            if len(self.buffer) < 7 + size:
                break
            frames.append(Frame(src, dst, self.buffer[7:7 + size]))
            del self.buffer[:7 + size]
        return frames


class ProtocolTests(unittest.TestCase):
    def test_replace_default_module_implementation(self):
        # This is the one implementation selected by frame in production.
        with patch('rcs_ptl.rcs_ptl_rdlc.ProtocolCodec', AlternateProtocol):
            tx, rx = FrameCodec(), FrameCodec()
            payload = bytes((0xC0, 0x0C, 0xFF)) * 100  # Larger than the RDLC default.
            raw = tx.encode(0x1234, 0x5678, payload)
            self.assertEqual(raw[:7], b'\xa5\x34\x12\x78\x56\x2c\x01')
            frames = []
            for start in range(0, len(raw), 5):
                frames.extend(rx.feed(raw[start:start + 5]))
            self.assertEqual(frames, [Frame(0x1234, 0x5678, payload)])

    def test_options_and_error_policy_are_delegated(self):
        with patch('rcs_ptl.rcs_ptl_rdlc.ProtocolCodec', AlternateProtocol):
            codec = FrameCodec(marker=0x7E)
            raw = codec.encode(300, 400, b'x')
            self.assertEqual(raw[0], 0x7E)
            self.assertEqual(codec.feed(b'noise' + raw), [Frame(300, 400, b'x')])
            with self.assertRaises(FrameDecodeError):
                codec.feed(b'bad', strict=True)

    def test_reset_and_parser_instances_are_independent(self):
        codec = FrameCodec(protocol=AlternateProtocol())
        other = FrameCodec(protocol=AlternateProtocol())
        raw = codec.encode(300, 400, b'x')
        self.assertEqual(codec.feed(raw[:3]), [])
        self.assertEqual(other.feed(raw), [Frame(300, 400, b'x')])
        codec.reset()
        self.assertEqual(codec.feed(raw[3:]), [])
        self.assertEqual(codec.feed(raw), [Frame(300, 400, b'x')])
