"""RDLC-over-serial link for the upper/lower computer connection test."""

from __future__ import annotations

import queue

from rcs_ptl.codec import RdlcCodec
from rdlc.frame import RdlcFrame

from .config import SerialConfig
from .transport import SerialTransport


class RdlcSerialLink:
    """Combine a threaded serial transport with a streaming RDLC codec."""

    def __init__(self, config: SerialConfig) -> None:
        self.config = config
        self.codec = RdlcCodec(config.max_payload_size, config.escape_enabled)
        self._frames: queue.Queue[RdlcFrame] = queue.Queue()
        self.transport = SerialTransport(config.port, config.baudrate, config.timeout, self._on_bytes)

    def open(self) -> None:
        self.transport.open()

    def close(self) -> None:
        self.transport.close()

    def send(self, payload: bytes, *, dst: int | None = None, src: int | None = None) -> bytes:
        frame = self.codec.encode(
            self.config.local_address if src is None else src,
            self.config.remote_address if dst is None else dst,
            payload,
        )
        self.transport.write(frame)
        return frame

    def receive(self, timeout: float | None = None) -> RdlcFrame:
        return self._frames.get(timeout=timeout)

    def _on_bytes(self, data: bytes) -> None:
        for frame in self.codec.feed(data):
            self._frames.put(frame)

