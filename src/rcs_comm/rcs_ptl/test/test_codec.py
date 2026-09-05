import unittest

from rcs_ptl import RdlcCodec, crc16_modbus


class TestRdlcCodec(unittest.TestCase):
    def test_crc_matches_modbus_vector(self):
        self.assertEqual(crc16_modbus(b"123456789"), 0x4B37)

    def test_mcu_default_frame_vector(self):
        codec = RdlcCodec(128, escape_enabled=False)
        raw = codec.encode(0x01, 0x02, bytes(range(1, 9)))
        self.assertEqual(raw.hex(" "), "c0 01 02 08 00 01 02 03 04 05 06 07 08 b0 cf 0c")

    def test_streaming_decode(self):
        codec = RdlcCodec()
        raw = codec.encode(1, 2, b"hello")
        frames = [frame for byte in raw for frame in codec.feed(bytes((byte,)))]
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload, b"hello")

    def test_escape_round_trip_for_control_bytes(self):
        codec = RdlcCodec(128, escape_enabled=True)
        payload = bytes((0x01, 0xC0, 0x0C, 0xFF, 0x02))
        raw = codec.encode(1, 2, payload)
        frames = codec.feed(raw)
        self.assertEqual(frames[0].payload, payload)

    def test_multiple_frames_and_partial_chunks(self):
        codec = RdlcCodec()
        raw = codec.encode(1, 2, b"a") + codec.encode(1, 2, b"b")
        frames = []
        for start in range(0, len(raw), 3):
            frames.extend(codec.feed(raw[start:start + 3]))
        self.assertEqual([frame.payload for frame in frames], [b"a", b"b"])


if __name__ == "__main__":
    unittest.main()
