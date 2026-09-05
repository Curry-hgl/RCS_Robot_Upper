"""协议适配层接口。"""

from .codec import RdlcCodec
from rdlc.crc import crc16_modbus
from rdlc.frame import RdlcFrame

# 对外公开的稳定 API：外部统一 from rcs_ptl import ... 使用
__all__ = ["RdlcCodec", "RdlcFrame", "crc16_modbus"]