"""RDLC 协议的完整定义：帧格式、长度、地址、CRC、转义及流式解析。

HEAD SRC DST LEN_LO LEN_HI PAYLOAD CRC_LO CRC_HI TAIL。
替换协议时只需替换本模块，并保留 ProtocolCodec 的构造及 encode/feed/reset 接口。
解析结果使用统一 Frame；协议字段限制在本模块检查。
"""
from __future__ import annotations

from enum import Enum, auto

from .rcs_ptl_frame import Frame, FrameDecodeError, FrameEncodeError


FRAME_ESCAPE = 0xFF  # 转义字节：转义模式下，数据里出现 FF/C0/0C 时前面补一个 FF
FRAME_HEAD = 0xC0    # 帧头：一帧的起始标记
FRAME_TAIL = 0x0C    # 帧尾：一帧的结束标记
DEFAULT_MAX_PAYLOAD = 128  # 帧载荷最大字节数（发送上限+接收防御上限）


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


class _State(Enum):
    """解码状态机的各阶段，代表"现在正在接收帧的哪个字段"。"""

    WAIT_HEAD = auto()  # 等待帧头
    SRC = auto()        # 正在收源地址
    DST = auto()        # 正在收目的地址
    LEN_LOW = auto()    # 正在收长度低位
    LEN_HIGH = auto()   # 正在收长度高位
    PAYLOAD = auto()    # 正在收载荷
    CRC_LOW = auto()    # 正在收校验值低位
    CRC_HIGH = auto()   # 正在收校验值高位
    TAIL = auto()       # 等待帧尾


class ProtocolCodec:
    """编码帧；对任意串口分片数据增量式解码（逐字节喂入）。"""

    def __init__(self, max_payload_size: int = DEFAULT_MAX_PAYLOAD,
                 escape_enabled: bool = False) -> None:
        if not 0 <= max_payload_size <= 0xFFFF - 6:
            raise ValueError("max_payload_size 超出范围")
        self.max_payload_size = max_payload_size
        self.escape_enabled = escape_enabled  # 转义模式开关（默认关，匹配 MCU 现状）
        self.reset()

    def reset(self) -> None:
        """回到初始状态，丢弃半截帧数据。"""
        self._state = _State.WAIT_HEAD
        self._escaped = False       # 当前字节是否被转义标记
        self._body = bytearray()    # 已收到的帧主体（src~crc）
        self._payload_size = 0      # 本帧载荷长度（由长度字段解析出来）
        self._crc_low = 0           # 收到的校验值低字节（暂存）

    def encode(self, src: int, dst: int, payload: bytes) -> bytes:
        """打包一帧完整的字节序列。"""
        payload = bytes(payload)
        if not 0 <= src <= 0xFF or not 0 <= dst <= 0xFF:
            raise FrameEncodeError("地址必须在 0..255 范围内")
        if len(payload) > self.max_payload_size:
            raise FrameEncodeError(
                f"载荷为 {len(payload)} 字节；最大为 {self.max_payload_size}")
        # 帧体 = src + dst + 长度(低字节在前，高字节在后) + 载荷
        body = bytes((src, dst, len(payload) & 0xFF, len(payload) >> 8)) + payload
        crc = crc16_modbus(payload)          # 校验值只对载荷计算
        body += bytes((crc & 0xFF, crc >> 8))  # 校验值也是低字节在前
        if not self.escape_enabled:
            return bytes((FRAME_HEAD,)) + body + bytes((FRAME_TAIL,))
        # 转义模式：帧头/帧尾前也各加一个转义字节，且帧内部
        # 出现 FF/C0/0C 时前面补一个 FF，避免和帧标记混淆。
        return bytes((FRAME_ESCAPE, FRAME_HEAD)) + self._escape(body) + bytes(
            (FRAME_ESCAPE, FRAME_TAIL))

    @staticmethod
    def _escape(data: bytes) -> bytes:
        """把帧体里的 FF/C0/0C 前面各插入一个转义字节 0xFF。"""
        encoded = bytearray()
        for byte in data:
            if byte in (FRAME_ESCAPE, FRAME_HEAD, FRAME_TAIL):
                encoded.append(FRAME_ESCAPE)
            encoded.append(byte)
        return bytes(encoded)

    def feed(self, data: bytes, *, strict: bool = False) -> list[Frame]:
        """逐字节解析串口数据，返回这一批数据解出的所有帧。

        strict=True：遇到坏帧立即抛 FrameDecodeError；
        否则跳过坏帧字节，继续尝试解析后续数据。
        """
        frames: list[Frame] = []
        for byte in bytes(data):
            try:
                frame = self._feed_byte(byte)
            except FrameDecodeError:
                self.reset()   # 帧已损坏，清空状态重新找帧头
                if strict:
                    raise
                continue
            if frame is not None:
                frames.append(frame)
        return frames

    def _feed_byte(self, byte: int) -> Frame | None:
        """处理单个字节，按状态推进；一帧收齐时返回 Frame。"""

        # ---- 转义模式的分支 ----
        if self.escape_enabled:
            if self._escaped:
                # 上一字节是 0xFF，说明当前字节是被转义的
                self._escaped = False
                if byte not in (FRAME_ESCAPE, FRAME_HEAD, FRAME_TAIL):
                    raise FrameDecodeError("无效的 RDLC 转义序列")
                # 还在等帧头：FF+C0 是"起始帧"，FF+0C 之后是帧尾
                if self._state is _State.WAIT_HEAD:
                    if byte != FRAME_HEAD:
                        raise FrameDecodeError("无效的转义帧头")
                    self._start_frame()
                    return None
                if self._state is _State.TAIL and byte == FRAME_TAIL:
                    return self._finish_frame()  # FF+0C 收尾，整帧完成
                # 帧已开始：被转义的字节按普通数据对待
                return self._consume_data(byte)
            if byte == FRAME_ESCAPE:
                self._escaped = True  # 记下"下一个字节是转义过的"
                return None
            if self._state is _State.WAIT_HEAD:
                return None           # 帧开始前的杂散字节直接忽略
            if byte == FRAME_HEAD:
                self._start_frame()   # 普通 0xC0 表示一帧开始
                return None
            if byte == FRAME_TAIL:
                raise FrameDecodeError("意外的帧尾")
            return self._consume_data(byte)

        # ---- 非转义模式的分支 ----
        if self._state is _State.WAIT_HEAD:
            if byte == FRAME_HEAD:
                self._start_frame()
            return None               # 帧头之外的字节忽略
        if byte == FRAME_HEAD:
            self._start_frame()       # 重入：前帧损坏，从帧头重新开始
            return None
        if self._state is _State.TAIL:
            if byte != FRAME_TAIL:
                raise FrameDecodeError("缺少 RDLC 帧尾")
            return self._finish_frame()
        if byte == FRAME_TAIL:
            raise FrameDecodeError("意外的 RDLC 帧尾")
        return self._consume_data(byte)

    def _start_frame(self) -> None:
        """收到帧头，清空上一帧残留，状态切到收 SRC。"""
        self._state = _State.SRC
        self._body.clear()
        self._payload_size = 0
        self._crc_low = 0

    def _consume_data(self, byte: int) -> Frame | None:
        """按当前状态意义地收一个字节，逐字段推进，完成校验。"""
        if self._state is _State.SRC:
            self._body.append(byte)
            self._state = _State.DST
        elif self._state is _State.DST:
            self._body.append(byte)
            self._state = _State.LEN_LOW
        elif self._state is _State.LEN_LOW:
            self._body.append(byte)
            self._payload_size = byte            # 长度低位
            self._state = _State.LEN_HIGH
        elif self._state is _State.LEN_HIGH:
            self._body.append(byte)
            self._payload_size |= byte << 8      # 长度高位拼进来（低字节在前）
            if self._payload_size > self.max_payload_size:
                raise FrameDecodeError("RDLC 载荷超过配置的最大值")
            # 长度字段收完：无载荷就直接收 CRC，否则收载荷
            self._state = _State.PAYLOAD if self._payload_size else _State.CRC_LOW
        elif self._state is _State.PAYLOAD:
            self._body.append(byte)
            # body 目前有 src+dst+len(共4字节)+已收载荷，凑够即转收 CRC
            if len(self._body) == 4 + self._payload_size:
                self._state = _State.CRC_LOW
        elif self._state is _State.CRC_LOW:
            self._crc_low = byte                 # 暂存校验低字节
            self._state = _State.CRC_HIGH
        elif self._state is _State.CRC_HIGH:
            received = self._crc_low | (byte << 8)  # 拼成完整 16 位校验值
            # 校验对象是 body[4:]（即 payload），与发送端一致
            if received != crc16_modbus(bytes(self._body[4:])):
                raise FrameDecodeError("RDLC 校验失败")
            self._state = _State.TAIL            # 校验通过，只差帧尾
        else:
            raise FrameDecodeError("无效的 RDLC 解析器状态")
        return None

    def _finish_frame(self) -> Frame:
        """帧尾收齐：取出 src/dst/payload 构造 Frame 并复位。"""
        if len(self._body) != 4 + self._payload_size:
            raise FrameDecodeError("RDLC 帧不完整")
        frame = Frame(self._body[0], self._body[1], self._body[4:])
        self.reset()  # 准备解析下一帧
        return frame
