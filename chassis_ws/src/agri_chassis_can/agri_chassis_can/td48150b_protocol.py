"""Pure protocol helpers for the TD48150B-2E dual servo driver.

The driver manual documents an 8-byte CAN payload carried in a 29-bit extended
CAN frame. This module deliberately contains no ROS or socket code so protocol
encoding can be unit-tested without hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


FRAME_LEN = 8
CHANNEL_A = 0x01
CHANNEL_B = 0x02
SPEED_MIN = -10000
SPEED_MAX = 10000


@dataclass(frozen=True)
class SpeedFeedback:
    """Raw speed feedback for driver channel A and B."""

    channel_a: int
    channel_b: int


@dataclass(frozen=True)
class FaultFeedback:
    """Raw 16-bit fault masks for driver channel A and B."""

    channel_a: int
    channel_b: int


def _validate_channel(channel: int) -> int:
    channel = int(channel)
    if channel not in (CHANNEL_A, CHANNEL_B):
        raise ValueError(f"channel must be 1 (A) or 2 (B), got {channel}")
    return channel


def _i32_be(value: int) -> bytes:
    value = int(value)
    if value < -(1 << 31) or value > (1 << 31) - 1:
        raise ValueError(f"int32 out of range: {value}")
    return value.to_bytes(4, byteorder="big", signed=True)


def _u16_be(data: bytes) -> int:
    if len(data) != 2:
        raise ValueError("u16 requires exactly two bytes")
    return int.from_bytes(data, byteorder="big", signed=False)


def _i16_be(data: bytes) -> int:
    if len(data) != 2:
        raise ValueError("i16 requires exactly two bytes")
    return int.from_bytes(data, byteorder="big", signed=True)


class TD48150BProtocol:
    """Encoder/decoder for the documented TD48150B-2E CAN messages."""

    @staticmethod
    def enable(channel: int) -> bytes:
        channel = _validate_channel(channel)
        return bytes((0x23, 0x0D, 0x20, channel, 0x00, 0x00, 0x00, 0x00))

    @staticmethod
    def disable(channel: int) -> bytes:
        channel = _validate_channel(channel)
        return bytes((0x23, 0x0C, 0x20, channel, 0x00, 0x00, 0x00, 0x00))

    @staticmethod
    def speed_command(channel: int, normalized_speed: int) -> bytes:
        """Build a speed setpoint frame.

        The documented normalized command range is -10000..10000, mapping to
        negative rated speed .. positive rated speed. Values outside that
        range are rejected instead of silently wrapping.
        """

        channel = _validate_channel(channel)
        normalized_speed = int(normalized_speed)
        if not SPEED_MIN <= normalized_speed <= SPEED_MAX:
            raise ValueError(
                f"normalized speed must be in [{SPEED_MIN}, {SPEED_MAX}], "
                f"got {normalized_speed}"
            )
        return bytes((0x23, 0x00, 0x20, channel)) + _i32_be(normalized_speed)

    @staticmethod
    def query_current() -> bytes:
        return bytes((0x40, 0x00, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00))

    @staticmethod
    def query_fault() -> bytes:
        return bytes((0x40, 0x12, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00))

    @staticmethod
    def query_speed() -> bytes:
        return bytes((0x40, 0x03, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00))

    @staticmethod
    def query_bus_voltage() -> bytes:
        return bytes((0x40, 0x0D, 0x21, 0x02, 0x00, 0x00, 0x00, 0x00))

    @staticmethod
    def query_temperature() -> bytes:
        return bytes((0x40, 0x0F, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00))

    @staticmethod
    def parse_speed_feedback(payload: bytes) -> Optional[SpeedFeedback]:
        if len(payload) != FRAME_LEN or payload[:4] != bytes((0x60, 0x03, 0x21, 0x01)):
            return None
        return SpeedFeedback(
            channel_a=_i16_be(payload[4:6]),
            channel_b=_i16_be(payload[6:8]),
        )

    @staticmethod
    def parse_fault_feedback(payload: bytes) -> Optional[FaultFeedback]:
        if len(payload) != FRAME_LEN or payload[:4] != bytes((0x60, 0x12, 0x21, 0x01)):
            return None
        return FaultFeedback(
            channel_a=_u16_be(payload[4:6]),
            channel_b=_u16_be(payload[6:8]),
        )

    @staticmethod
    def parse_current_feedback(payload: bytes) -> Optional[tuple[int, int]]:
        if len(payload) != FRAME_LEN or payload[:4] != bytes((0x60, 0x00, 0x21, 0x01)):
            return None
        return _u16_be(payload[4:6]), _u16_be(payload[6:8])

    @staticmethod
    def parse_bus_voltage_feedback(payload: bytes) -> Optional[int]:
        if len(payload) != FRAME_LEN or payload[:4] != bytes((0x60, 0x0D, 0x21, 0x02)):
            return None
        return _u16_be(payload[6:8])

    @staticmethod
    def parse_temperature_feedback(payload: bytes) -> Optional[tuple[int, int, int]]:
        if len(payload) != FRAME_LEN or payload[:4] != bytes((0x60, 0x0F, 0x21, 0x01)):
            return None
        return payload[5], payload[6], payload[7]
