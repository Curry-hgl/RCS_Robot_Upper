"""RDLC 帧的值对象。"""

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class RdlcFrame:
    """一帧 RDLC 数据：src 源地址 / dst 目的地址 / payload 业务载荷。"""

    src: int
    dst: int
    payload: bytes

    def __post_init__(self) -> None:
        # dataclass 构造后自动调用，做合法性校验
        if not 0 <= self.src <= 0xFF or not 0 <= self.dst <= 0xFF:
            raise ValueError("RDLC 地址必须在 0..255 范围内")
        # frozen=True 不能直接赋值，用底层方法把 payload 统一转成 bytes
        object.__setattr__(self, "payload", bytes(self.payload))