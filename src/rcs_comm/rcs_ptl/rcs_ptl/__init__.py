"""RCS 协议无关的帧接口。"""
from .rcs_ptl_frame import (
    Frame, FrameCodec, FrameProtocol, FrameError, FrameEncodeError, FrameDecodeError,
)

__all__ = [
    'Frame', 'FrameCodec', 'FrameProtocol', 'FrameError', 'FrameEncodeError', 'FrameDecodeError',
]
