"""业务载荷验证与编解码；不注册消息，RDLC 封帧属于 rcs_ptl。"""
import math
import re
import struct
from dataclasses import dataclass

VISION = 0x21  # 通用三维视觉；0x01 用于批量物块
CHASSIS = 0x20  # 底盘控制；0x02 用于黑环
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


def encode_chassis(msg):
    return bytes((CHASSIS,)) + struct.pack(
        "<BfffB", msg.mode, msg.vx, msg.vy, msg.wz, msg.priority)


def encode_vision(msg):
    return bytes((VISION,)) + struct.pack(
        "<Bfff", {"block": 1, "cylinder": 2}.get(msg.object_id, 0),
        msg.position_x, msg.position_y, msg.position_z)


BLOCKS, RINGS, TASK_CODE, MODE, GRIPPER = range(1, 6)


def valid_task_code(raw):
    """严格四组三位 1..6，拒绝截断、非法数字和额外字符。"""
    return re.fullmatch(r"[1-6]{3}(?:\+[1-6]{3}){3}", raw) is not None


def encode_detections(msg):
    """输出 类型+数量+最多三个 <颜色u8,cx f32,cy f32,r f32>。"""
    if msg.mode not in (0, 1):
        raise ValueError("Invalid detection mode")
    objects = list(msg.objects)[:3]
    raw = bytes((BLOCKS if msg.mode == 0 else RINGS, len(objects)))
    for obj in objects:
        if not 1 <= obj.color_id <= 6 or not all(
                math.isfinite(v) and v >= 0 for v in (obj.cx, obj.cy, obj.radius)):
            raise ValueError("Invalid detection")
        raw += struct.pack("<Bfff", obj.color_id, obj.cx, obj.cy, obj.radius)
    return raw


def encode_task_code(msg):
    if not valid_task_code(msg.raw):
        raise ValueError("Invalid task code")
    return bytes((TASK_CODE,)) + msg.raw.encode("ascii")


@dataclass(frozen=True)
class ModeCommand:
    mode: int


@dataclass(frozen=True)
class GripperEvent:
    action: int
    pick: int = 0
    place: int = 0


def decode_mode(raw):
    if len(raw) != 2 or raw[0] != MODE or raw[1] not in (0, 1):
        raise ValueError("Invalid mode command")
    return ModeCommand(raw[1])


def decode_gripper(raw):
    if len(raw) < 2 or raw[0] != GRIPPER:
        raise ValueError("Invalid gripper event")
    if raw[1] == 0x10 and len(raw) == 4:
        return GripperEvent(raw[1], raw[2], raw[3])
    if raw[1] in (1, 2, 3, 4, 5) and len(raw) == 2:
        return GripperEvent(raw[1])
    raise ValueError("Invalid gripper operation or length")
