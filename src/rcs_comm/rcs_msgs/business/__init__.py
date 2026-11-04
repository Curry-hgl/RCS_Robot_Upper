"""业务载荷编解码和注册表的公共接口，不依赖 ROS 生成类型。"""
from .rcs_msgs_payload import (
    VISION, CHASSIS, ACTION_COMMAND, ACTION_FEEDBACK, VERSION, MAX_DATA,
    BLOCKS, RINGS, TASK_CODE, MODE, GRIPPER,
    Command, Feedback, ModeCommand, GripperEvent,
    encode_command, decode_command, encode_feedback, decode_feedback,
    encode_chassis, encode_vision, encode_detections, encode_task_code,
    decode_mode, decode_gripper, valid_task_code,
)
from .rcs_msgs_registry import Registry, default_registry
