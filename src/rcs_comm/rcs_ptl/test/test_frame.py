import unittest

from rcs_ptl import FrameCodec, FrameEncodeError, FrameDecodeError
from rcs_ptl.rcs_ptl_rdlc import crc16_modbus


class TestFrameCodec(unittest.TestCase):
    def test_crc_matches_modbus_vector(self):
        self.assertEqual(crc16_modbus(b"123456789"), 0x4B37)

    def test_mcu_default_frame_vector(self):
        codec = FrameCodec(max_payload_size=128, escape_enabled=False)
        raw = codec.encode(0x01, 0x02, bytes(range(1, 9)))
        self.assertEqual(raw.hex(" "), "c0 01 02 08 00 01 02 03 04 05 06 07 08 b0 cf 0c")

    def test_streaming_decode(self):
        codec = FrameCodec()
        raw = codec.encode(1, 2, b"hello")
        frames = [frame for byte in raw for frame in codec.feed(bytes((byte,)))]
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload, b"hello")

    def test_escape_round_trip_for_control_bytes(self):
        codec = FrameCodec(max_payload_size=128, escape_enabled=True)
        payload = bytes((0x01, 0xC0, 0x0C, 0xFF, 0x02))
        raw = codec.encode(1, 2, payload)
        frames = codec.feed(raw)
        self.assertEqual(frames[0].payload, payload)

    def test_multiple_frames_and_partial_chunks(self):
        codec = FrameCodec()
        raw = codec.encode(1, 2, b"a") + codec.encode(1, 2, b"b")
        frames = []
        for start in range(0, len(raw), 3):
            frames.extend(codec.feed(raw[start:start + 3]))
        self.assertEqual([frame.payload for frame in frames], [b"a", b"b"])

    def test_crc_failure_and_recovery(self):
        codec = FrameCodec()
        good = codec.encode(1, 2, b'hello')
        bad = bytearray(good)
        bad[5] ^= 1  # Change payload without updating the checksum.
        self.assertEqual(codec.feed(bytes(bad) + good)[0].payload, b'hello')
        with self.assertRaises(FrameDecodeError):
            codec.feed(bad, strict=True)
        self.assertEqual(codec.feed(good)[0].payload, b'hello')

    def test_rdlc_limits_and_empty_payload(self):
        codec = FrameCodec(max_payload_size=4)
        with self.assertRaises(FrameEncodeError):
            codec.encode(256, 2, b'')
        with self.assertRaises(FrameEncodeError):
            codec.encode(1, 2, b'12345')
        self.assertEqual(codec.feed(codec.encode(1, 2, b''))[0].payload, b'')


if __name__ == "__main__":
    unittest.main()
