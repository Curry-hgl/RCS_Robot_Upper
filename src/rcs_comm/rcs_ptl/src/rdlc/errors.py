"""RDLC 协议异常。"""


class RdlcError(Exception):
    """协议异常基类：捕获它等于捕获所有 RDLC 错误。"""


class RdlcEncodeError(RdlcError, ValueError):
    """编码出错：地址越界、载荷超长等。"""


class RdlcDecodeError(RdlcError, ValueError):
    """解码出错：头尾不匹配、CRC 校验失败、载荷超长、转义非法等。"""