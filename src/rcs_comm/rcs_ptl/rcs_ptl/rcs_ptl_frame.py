"""与线协议无关的帧对象和收发入口；所有线格式规则由协议模块定义。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class FrameError(ValueError):
    """帧处理错误。"""


class FrameEncodeError(FrameError):
    """发送内容无法按所选协议编码。"""


class FrameDecodeError(FrameError):
    """接收数据不符合所选协议。"""


@dataclass(frozen=True, slots=True)
class Frame:
    """统一交换对象；地址在这里不限定宽度，由协议实现验证。"""

    src: int
    dst: int
    payload: bytes

    def __post_init__(self) -> None:
        object.__setattr__(self, 'payload', bytes(self.payload))


class FrameProtocol(Protocol):
    """协议实现契约：每个实例持有自己的分片接收状态。"""

    def encode(self, src: int, dst: int, payload: bytes) -> bytes: ...
    def feed(self, data: bytes, *, strict: bool = False) -> list[Frame]: ...
    def reset(self) -> None: ...


class FrameCodec:
    """统一封帧/拆帧接口；不解释帧头、长度、校验或转义规则。"""

    def __init__(self, *, protocol: FrameProtocol | None = None,
                 **protocol_options) -> None:
        if protocol is not None:
            if protocol_options:
                raise ValueError('自定义协议的参数应在创建 protocol 实例时设置')
            self._protocol = protocol
        else:
            # 唯一默认协议入口；参数原样传递，具体含义和默认值由协议定义。
            from .rcs_ptl_rdlc import ProtocolCodec
            self._protocol = ProtocolCodec(**protocol_options)

    def encode(self, src: int, dst: int, payload: bytes) -> bytes:
        return self._protocol.encode(src, dst, payload)

    def feed(self, data: bytes, *, strict: bool = False) -> list[Frame]:
        return self._protocol.feed(data, strict=strict)

    def reset(self) -> None:
        self._protocol.reset()
