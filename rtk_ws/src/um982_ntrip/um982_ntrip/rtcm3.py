"""Small RTCM3 stream validator used by the UM982 NTRIP bridge."""

from collections import Counter
from typing import Dict, List


def crc24q(data: bytes) -> int:
    """Return the RTCM3 CRC-24Q value for *data*."""
    crc = 0
    polynomial = 0x1864CFB
    for value in data:
        crc ^= value << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= polynomial
        crc &= 0xFFFFFF
    return crc


class Rtcm3Inspector:
    """Recover and validate RTCM3 frames from arbitrary TCP chunks."""

    def __init__(self) -> None:
        """Initialize an empty cumulative stream inspection state."""
        self._buffer = bytearray()
        self.valid_frames = 0
        self.invalid_crc = 0
        self.invalid_header = 0
        self.discarded_bytes = 0
        self.message_types: Counter[int] = Counter()

    def feed(self, data: bytes) -> List[bytes]:
        """Consume bytes and return complete frames with valid CRC-24Q."""
        self._buffer.extend(data)
        frames = []

        while self._buffer:
            preamble = self._buffer.find(0xD3)
            if preamble < 0:
                self.discarded_bytes += len(self._buffer)
                self._buffer.clear()
                break
            if preamble > 0:
                self.discarded_bytes += preamble
                del self._buffer[:preamble]
            if len(self._buffer) < 3:
                break
            if self._buffer[1] & 0xFC:
                self.invalid_header += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue

            payload_length = (
                ((self._buffer[1] & 0x03) << 8) | self._buffer[2])
            frame_length = 3 + payload_length + 3
            if len(self._buffer) < frame_length:
                break

            frame = bytes(self._buffer[:frame_length])
            expected_crc = int.from_bytes(frame[-3:], byteorder='big')
            if crc24q(frame[:-3]) != expected_crc:
                self.invalid_crc += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue

            payload = frame[3:-3]
            if len(payload) >= 2:
                message_type = (payload[0] << 4) | (payload[1] >> 4)
                self.message_types[message_type] += 1
            self.valid_frames += 1
            frames.append(frame)
            del self._buffer[:frame_length]

        return frames

    def status(self) -> Dict[str, object]:
        """Return JSON-serializable cumulative stream diagnostics."""
        return {
            'valid_frames': self.valid_frames,
            'invalid_crc': self.invalid_crc,
            'invalid_header': self.invalid_header,
            'discarded_bytes': self.discarded_bytes,
            'message_types': {
                str(key): self.message_types[key]
                for key in sorted(self.message_types)
            },
        }
