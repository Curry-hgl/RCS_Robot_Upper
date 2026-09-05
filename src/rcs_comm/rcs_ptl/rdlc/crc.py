"""与 RCS_RDLC.c 一致的 CRC 实现。"""


def crc16_modbus(data: bytes) -> int:
    """返回 CRC-16/Modbus 校验值（初始 0xffff，多项式 0xa001）。"""
    crc = 0xFFFF                 # 寄存器初始值
    for byte in data:
        crc ^= byte              # 字节异或进寄存器低 8 位
        for _ in range(8):       # 逐位处理该字节的 8 个 bit
            if crc & 1:          # 最低位为 1：右移后异或多项式纠错
                crc = (crc >> 1) ^ 0xA001
            else:                # 最低位为 0：只右移
                crc = crc >> 1
    return crc & 0xFFFF          # 掩成 16 位无符号数返回