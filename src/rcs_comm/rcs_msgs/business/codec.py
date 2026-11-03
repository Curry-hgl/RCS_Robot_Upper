"""Little-endian business payload registry; RDLC framing belongs to rcs_ptl."""
import struct
from dataclasses import dataclass

VISION = 0x01
CHASSIS = 0x02
ACTION_COMMAND = 0x10
ACTION_FEEDBACK = 0x11
# Version 1: reject incompatible peers rather than silently reinterpret fields.
VERSION = 1
MAX_DATA = 116  # feedback header is 11 bytes; RDLC default payload limit is 128


@dataclass(frozen=True)
class Command:
    request_id: int
    action_code: int
    operation: int = 0
    data: bytes = b""


@dataclass(frozen=True)
class Feedback:
    request_id: int
    action_code: int
    status: int
    error_code: int = 0
    data: bytes = b""


def _data(data):
    data = bytes(data)
    if len(data) > MAX_DATA:
        raise ValueError("Business data exceeds 116 bytes")
    return data


def encode_command(msg):
    if msg.request_id == 0 or msg.operation not in (0, 1):
        raise ValueError("Invalid request ID or operation")
    return struct.pack("<BBIHB", ACTION_COMMAND, VERSION, msg.request_id,
                       msg.action_code, msg.operation) + _data(msg.data)


def decode_command(raw):
    if len(raw) < 9 or raw[:2] != bytes((ACTION_COMMAND, VERSION)):
        raise ValueError("Invalid command header or length")
    request, action, operation = struct.unpack("<IHB", raw[2:9])
    msg = Command(request, action, operation, _data(raw[9:]))
    encode_command(msg)
    return msg


def encode_feedback(msg):
    if msg.request_id == 0 or msg.status not in range(5):
        raise ValueError("Invalid request ID or status")
    return struct.pack("<BBIHBH", ACTION_FEEDBACK, VERSION, msg.request_id,
                       msg.action_code, msg.status, msg.error_code) + _data(msg.data)


def decode_feedback(raw):
    if len(raw) < 11 or raw[:2] != bytes((ACTION_FEEDBACK, VERSION)):
        raise ValueError("Invalid feedback header or length")
    request, action, status, error = struct.unpack("<IHBH", raw[2:11])
    msg = Feedback(request, action, status, error, _data(raw[11:]))
    encode_feedback(msg)
    return msg


class Registry:
    def __init__(self):
        self._encoders = {}
        self._decoders = {}

    def register(self, message_id, encoder, decoder=None):
        if message_id in self._encoders:
            raise ValueError("Duplicate message ID")
        self._encoders[message_id] = encoder
        if decoder is not None:
            self._decoders[message_id] = decoder

    def encode(self, message_id, msg):
        return self._encoders[message_id](msg)

    def decode(self, raw):
        if not raw or raw[0] not in self._decoders:
            raise ValueError("Unknown incoming message ID")
        return self._decoders[raw[0]](raw)


def default_registry():
    registry = Registry()
    registry.register(ACTION_COMMAND, encode_command, decode_command)
    registry.register(ACTION_FEEDBACK, encode_feedback, decode_feedback)
    registry.register(CHASSIS, lambda m: bytes((CHASSIS,)) + struct.pack(
        "<BfffB", m.mode, m.vx, m.vy, m.wz, m.priority))
    registry.register(VISION, lambda m: bytes((VISION,)) + struct.pack(
        "<Bfff", {"block": 1, "cylinder": 2}.get(m.object_id, 0),
        m.position_x, m.position_y, m.position_z))
    return registry


# Replaceable season example: action 1 = lift, argument = target height in metres.
EXAMPLE_LIFT = 1

def lift_parameters(height):
    import math
    if not math.isfinite(height) or height < 0:
        raise ValueError("Height must be finite and nonnegative")
    return struct.pack("<f", height)
