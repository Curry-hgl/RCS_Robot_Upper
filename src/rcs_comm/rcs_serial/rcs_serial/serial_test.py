"""RDLC 回环测试：收到一帧，原样发回给发送方；Ctrl+C 退出。"""

from __future__ import annotations

import argparse
from queue import Empty

from .config import SerialConfig
from .rdlc_link import RdlcSerialLink


def main() -> int:
    parser = argparse.ArgumentParser(
        description="RDLC echo: 持续监听，收到一帧后原样发回，Ctrl+C 退出")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--src", type=lambda x: int(x, 0), default=0x01)
    parser.add_argument("--dst", type=lambda x: int(x, 0), default=0x02)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--escape", action="store_true")
    args = parser.parse_args()

    config = SerialConfig(args.port, args.baudrate, 0.1,
                          args.src, args.dst, 128, args.escape)
    link = RdlcSerialLink(config)
    link.open()
    print(f"echo 监听中：{config.port}@{config.baudrate}，Ctrl+C 退出")
    try:
        while True:
            try:
                frame = link.receive(timeout=args.timeout)
            except Empty:
                continue  # 超时没帧，继续等
            raw = link.send(frame.payload, dst=frame.src, src=frame.dst)
            print(f"RX: src=0x{frame.src:02X} dst=0x{frame.dst:02X} "
                  f"payload={frame.payload.hex(' ')}")
            print(f"TX: {raw.hex(' ')}")
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，退出")
        return 0
    finally:
        link.close()