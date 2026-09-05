"""Serial link configuration."""

from dataclasses import dataclass

from rdlc.constants import DEFAULT_BAUDRATE, DEFAULT_MAX_PAYLOAD


@dataclass(frozen=True, slots=True)
class SerialConfig:
    port: str
    baudrate: int = DEFAULT_BAUDRATE
    timeout: float = 0.1
    local_address: int = 0x01
    remote_address: int = 0x02
    max_payload_size: int = DEFAULT_MAX_PAYLOAD
    escape_enabled: bool = False

