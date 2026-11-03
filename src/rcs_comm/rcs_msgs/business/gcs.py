"""工创赛业务字节契约：沿用旧下位机格式，不包含 RDLC 帧头和 CRC。

本模块只做数据验证和编解码，不依赖 ROS 生成类型，便于上、下位机对表测试。
"""
import math
import re
import struct
from dataclasses import dataclass

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


def register_gcs(registry):
    """集中注册工创编号，驱动不拥有业务编码规则。"""
    registry.register(BLOCKS, encode_detections)
    registry.register(RINGS, encode_detections)
    registry.register(TASK_CODE, encode_task_code)
    registry.register(MODE, None, decode_mode)
    registry.register(GRIPPER, None, decode_gripper)
