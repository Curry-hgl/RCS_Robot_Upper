"""Protocol-independent framed serial link."""

from __future__ import annotations

import queue

from rcs_ptl import FrameCodec, Frame

from .rcs_serial_config import SerialConfig
from .rcs_serial_transport import SerialTransport


class SerialLink:
    """Combine a threaded serial transport with the generic frame interface."""

    def __init__(self, config: SerialConfig) -> None:
        self.config = config
        options = {name: value for name, value in {
            'max_payload_size': config.max_payload_size,
            'escape_enabled': config.escape_enabled,
        }.items() if value is not None}
        self.codec = FrameCodec(**options)
        self._frames: queue.Queue[Frame] = queue.Queue()
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

    def receive(self, timeout: float | None = None) -> Frame:
        return self._frames.get(timeout=timeout)

    def _on_bytes(self, data: bytes) -> None:
        for frame in self.codec.feed(data):
            self._frames.put(frame)

