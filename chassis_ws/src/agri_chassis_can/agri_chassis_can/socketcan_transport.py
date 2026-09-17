"""Minimal Linux SocketCAN transport for classical CAN frames."""

from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
from typing import Optional


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes
    is_extended: bool
    is_error: bool = False
    is_remote: bool = False


class SocketCanTransport:
    """Non-blocking classical CAN transport using only Python stdlib."""

    def __init__(self, interface: str) -> None:
        self.interface = str(interface)
        self.sock: Optional[socket.socket] = None

    def open(self) -> None:
        if self.sock is not None:
            return
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((self.interface,))
        sock.setblocking(False)
        self.sock = sock

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def send(self, can_id: int, data: bytes, *, extended: bool = True) -> None:
        if self.sock is None:
            raise RuntimeError("SocketCAN transport is not open")
        if len(data) > 8:
            raise ValueError("classical CAN payload cannot exceed 8 bytes")

        if extended:
            raw_can_id = (int(can_id) & CAN_EFF_MASK) | CAN_EFF_FLAG
        else:
            raw_can_id = int(can_id) & CAN_SFF_MASK

        packed = struct.pack(
            CAN_FRAME_FMT,
            raw_can_id,
            len(data),
            bytes(data).ljust(8, b"\x00"),
        )
        self.sock.send(packed)

    def recv_one(self) -> Optional[CanFrame]:
        if self.sock is None:
            raise RuntimeError("SocketCAN transport is not open")
        try:
            packet = self.sock.recv(CAN_FRAME_SIZE)
        except BlockingIOError:
            return None

        if len(packet) != CAN_FRAME_SIZE:
            return None

        raw_can_id, dlc, raw_data = struct.unpack(CAN_FRAME_FMT, packet)
        is_extended = bool(raw_can_id & CAN_EFF_FLAG)
        is_error = bool(raw_can_id & CAN_ERR_FLAG)
        is_remote = bool(raw_can_id & CAN_RTR_FLAG)
        can_id = raw_can_id & (CAN_EFF_MASK if is_extended else CAN_SFF_MASK)
        return CanFrame(
            can_id=can_id,
            data=raw_data[: min(int(dlc), 8)],
            is_extended=is_extended,
            is_error=is_error,
            is_remote=is_remote,
        )

    def recv_many(self, max_frames: int = 128) -> list[CanFrame]:
        frames: list[CanFrame] = []
        for _ in range(max(1, int(max_frames))):
            frame = self.recv_one()
            if frame is None:
                break
            frames.append(frame)
        return frames
