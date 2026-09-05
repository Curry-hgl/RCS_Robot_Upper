"""RDLC 协议基础模块：常量、异常、CRC、帧对象。"""

from .constants import (
    DEFAULT_BAUDRATE,
    DEFAULT_MAX_PAYLOAD,
    FRAME_ESCAPE,
    FRAME_HEAD,
    FRAME_TAIL,
)
from .crc import crc16_modbus
from .errors import RdlcDecodeError, RdlcEncodeError, RdlcError
from .frame import RdlcFrame

__all__ = [
    "DEFAULT_BAUDRATE",
    "DEFAULT_MAX_PAYLOAD",
    "FRAME_ESCAPE",
    "FRAME_HEAD",
    "FRAME_TAIL",
    "crc16_modbus",
    "RdlcDecodeError",
    "RdlcEncodeError",
    "RdlcError",
    "RdlcFrame",
]