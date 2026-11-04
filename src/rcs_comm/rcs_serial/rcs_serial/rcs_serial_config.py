"""Serial link configuration."""

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class SerialConfig:
    port: str
    baudrate: int = 115200
    timeout: float = 0.1
    local_address: int = 0x01
    remote_address: int = 0x02
    max_payload_size: int | None = None
    escape_enabled: bool | None = None

