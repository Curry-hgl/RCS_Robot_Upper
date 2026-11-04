"""业务编号与编解码函数的注册映射；不实现载荷编解码。"""
from .rcs_msgs_payload import (
    ACTION_COMMAND, ACTION_FEEDBACK, CHASSIS, VISION,
    BLOCKS, RINGS, TASK_CODE, MODE, GRIPPER,
    encode_command, decode_command, encode_feedback, decode_feedback,
    encode_chassis, encode_vision, encode_detections, encode_task_code,
    decode_mode, decode_gripper,
)


class Registry:
    def __init__(self):
        self._encoders = {}
        self._decoders = {}

    def register(self, message_id, encoder, decoder=None):
        """绑定唯一编号和收发函数；只接收的业务可不提供 encoder。"""
        if message_id in self._encoders:
            raise ValueError("Duplicate message ID")
        self._encoders[message_id] = encoder
        if decoder is not None:
            self._decoders[message_id] = decoder

    def encode(self, message_id, msg):
        encoder = self._encoders[message_id]
        if encoder is None:
            raise ValueError("Receive-only message")
        return encoder(msg)

    def decode(self, raw):
        if not raw or raw[0] not in self._decoders:
            raise ValueError("Unknown incoming message ID")
        return self._decoders[raw[0]](raw)


def default_registry():
    """集中绑定业务编号和编解码函数；驱动通过此表收发。"""
    registry = Registry()
    registry.register(ACTION_COMMAND, encode_command, decode_command)
    registry.register(ACTION_FEEDBACK, encode_feedback, decode_feedback)
    registry.register(CHASSIS, encode_chassis)
    registry.register(VISION, encode_vision)
    registry.register(BLOCKS, encode_detections)
    registry.register(RINGS, encode_detections)
    registry.register(TASK_CODE, encode_task_code)
    registry.register(MODE, None, decode_mode)
    registry.register(GRIPPER, None, decode_gripper)
    return registry
