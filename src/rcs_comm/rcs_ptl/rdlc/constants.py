"""RCS RDLC 编解码器共享的协议常量。"""

FRAME_ESCAPE = 0xFF  # 转义字节：转义模式下，数据里出现 FF/C0/0C 时前面补一个 FF
FRAME_HEAD = 0xC0    # 帧头：一帧的起始标记
FRAME_TAIL = 0x0C    # 帧尾：一帧的结束标记
DEFAULT_BAUDRATE = 115200  # 默认串口波特率
DEFAULT_MAX_PAYLOAD = 128  # 帧载荷最大字节数（发送上限+接收防御上限）