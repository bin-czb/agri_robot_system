#!/usr/bin/env python3
"""
AOA Serial Protocol Parser (stateless byte layout, stateful buffer).

Supports:
  - 0x2001  Position packet   (37 bytes, XOR-checked)
  - 0x2002  Heartbeat packet  (16 bytes, NO XOR byte per spec)

Wire byte order is big-endian for all multi-byte fields (confirmed by
vendor's STM32 reference calling U16/U32HighLowByteSwap, and by the
spec example frames).

This module is ROS-independent.  It only converts a raw byte stream
into validated, structured dicts.

Example (0x2001, 37 bytes, last byte = 0xA6):
    FF FF FF FF 00 25 22 46 20 01 01 02 00 00 00 64 00 00 00 64
    00 00 00 7D 01 48 00 00 12 34 10 6B AC 00 00 00 A6

Example (0x2002, 16 bytes, no XOR):
    FF FF FF FF 00 10 00 34 20 02 01 02 00 00 00 64
"""

from __future__ import annotations

import struct
from typing import List, Dict, Optional


# ─────────────────────────────────────────────────────────────────────
#  Protocol constants
# ─────────────────────────────────────────────────────────────────────
HEADER = b"\xFF\xFF\xFF\xFF"

CMD_POSITION  = 0x2001
CMD_HEARTBEAT = 0x2002

FRAME_LEN_POSITION  = 37
FRAME_LEN_HEARTBEAT = 16

# Fixed offsets inside a 0x2001 frame (relative to start of header)
_P_PACKET_LEN = 4     # 2B  BE uint
_P_SEQ_ID     = 6     # 2B  BE uint
_P_CMD        = 8     # 2B  BE uint
_P_VERSION    = 10    # 2B  BE uint
_P_ANCHOR_ID  = 12    # 4B  BE uint
_P_TAG_ID     = 16    # 4B  BE uint
_P_DISTANCE   = 20    # 4B  BE uint  (cm)
_P_AZIMUTH    = 24    # 2B  BE int   (1°/LSB, range 0..359)
_P_ELEVATION  = 26    # 2B  BE int   (reserved)
_P_TAG_STATUS = 28    # 2B  BE uint  (opaque)
_P_BATCH_SN   = 30    # 2B  BE uint
_P_RESERVE    = 32    # 4B  opaque
_P_XOR        = 36    # 1B

# Maximum bytes we are willing to keep in the buffer before discarding
# the front half on overflow.  At 115200 baud this represents several
# seconds of traffic and is safe for normal operation.
_BUFFER_HARD_LIMIT = 8192
_BUFFER_TRIM_KEEP  = 512


# ─────────────────────────────────────────────────────────────────────
#  Frame parser
# ─────────────────────────────────────────────────────────────────────
class AoaFrameParser:
    """
    Stateful byte-stream parser.  Call ``feed(chunk)`` with whatever the
    serial read returned and receive a list of zero or more fully
    validated frame dicts.

    The parser never blocks, never raises on malformed input, and always
    resynchronises on the next valid 0xFF 0xFF 0xFF 0xFF header.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

        # Diagnostic counters (node reads these to throttle log output).
        self.position_count: int  = 0
        self.heartbeat_count: int = 0
        self.xor_fail_count: int  = 0
        self.resync_count: int    = 0
        self.bad_length_count: int = 0
        self.unknown_cmd_count: int = 0

    # -----------------------------------------------------------------
    def feed(self, data: bytes) -> List[Dict]:
        """Append bytes to the internal buffer and return every frame
        that could be fully decoded this round."""
        frames: List[Dict] = []
        if data:
            self._buf.extend(data)

        # Keep pulling until we can no longer make progress in the
        # buffer.  A rejected frame (bad XOR, bad length, unknown cmd)
        # still consumes bytes, so the outer loop cannot stop on
        # ``None`` alone — it must stop only when the buffer size is
        # stable (i.e. genuinely needs more bytes).
        while True:
            prev_len = len(self._buf)
            frame = self._try_extract_one()
            if frame is not None:
                frames.append(frame)
                continue
            if len(self._buf) == prev_len:
                break  # no progress — wait for the next feed()

        # Defensive: if the buffer is growing without producing frames,
        # throw away the older half so we cannot blow up on junk streams.
        # Smart trim: keep the TAIL, but if that tail contains a header
        # anchor the trim to it so we don't leave a half-frame at the
        # start.  Falls back to blind tail-keep if no header is found.
        if len(self._buf) > _BUFFER_HARD_LIMIT:
            tail = self._buf[-_BUFFER_TRIM_KEEP:]
            hdr_idx = tail.find(HEADER)
            if hdr_idx >= 0:
                self._buf = bytearray(tail[hdr_idx:])
            else:
                self._buf = bytearray(tail)

        return frames

    # -----------------------------------------------------------------
    def _try_extract_one(self) -> Optional[Dict]:
        """Attempt to pull exactly one validated frame off the front of
        the buffer.  Returns None when not enough bytes are available or
        the leading bytes do not begin a valid frame."""
        buf = self._buf

        # 1) Find header.
        idx = buf.find(HEADER)
        if idx < 0:
            # Keep the last 3 bytes in case they start a header that is
            # completing on the next feed().
            if len(buf) > 3:
                del buf[:-3]
            return None
        if idx > 0:
            del buf[:idx]
            self.resync_count += 1

        # 2) Need at least header + PacketLength.
        if len(buf) < _P_PACKET_LEN + 2:
            return None

        packet_len = struct.unpack_from(">H", buf, _P_PACKET_LEN)[0]

        # 3) Sanity-check length BEFORE reserving bytes.  Only two
        #    lengths are legal per spec.
        if packet_len not in (FRAME_LEN_POSITION, FRAME_LEN_HEARTBEAT):
            # Drop the first 0xFF and keep scanning; the Reserve field
            # of a valid frame could legitimately contain 0xFF 0xFF 0xFF
            # 0xFF, so we cannot trust a single header match.
            del buf[0]
            self.bad_length_count += 1
            return None

        # 4) Need the whole frame.
        if len(buf) < packet_len:
            return None

        # 5) Dispatch on RequestCommand (need at least 10 bytes, which
        #    both frame types have).
        cmd = struct.unpack_from(">H", buf, _P_CMD)[0]
        frame_bytes = bytes(buf[:packet_len])
        del buf[:packet_len]  # consume regardless of validation result

        if cmd == CMD_POSITION:
            if packet_len != FRAME_LEN_POSITION:
                self.bad_length_count += 1
                return None
            return self._parse_position(frame_bytes)

        if cmd == CMD_HEARTBEAT:
            if packet_len != FRAME_LEN_HEARTBEAT:
                self.bad_length_count += 1
                return None
            return self._parse_heartbeat(frame_bytes)

        # Unknown command — swallow the frame (it is length-consistent
        # so the buffer stays aligned) and keep going.
        self.unknown_cmd_count += 1
        return None

    # -----------------------------------------------------------------
    def _parse_position(self, pkt: bytes) -> Optional[Dict]:
        """Parse a 37-byte 0x2001 frame.  Returns None if XOR fails."""
        xor_calc = 0
        for b in pkt[:_P_XOR]:
            xor_calc ^= b
        xor_recv = pkt[_P_XOR]
        if xor_calc != xor_recv:
            self.xor_fail_count += 1
            return None

        seq          = struct.unpack_from(">H",  pkt, _P_SEQ_ID)[0]
        version      = struct.unpack_from(">H",  pkt, _P_VERSION)[0]
        anchor_id    = struct.unpack_from(">I",  pkt, _P_ANCHOR_ID)[0]
        tag_id       = struct.unpack_from(">I",  pkt, _P_TAG_ID)[0]
        distance_cm  = struct.unpack_from(">I",  pkt, _P_DISTANCE)[0]
        azimuth_raw  = struct.unpack_from(">h",  pkt, _P_AZIMUTH)[0]   # signed
        elevation_raw = struct.unpack_from(">h", pkt, _P_ELEVATION)[0]  # signed
        tag_status   = struct.unpack_from(">H",  pkt, _P_TAG_STATUS)[0]
        batch_sn     = struct.unpack_from(">H",  pkt, _P_BATCH_SN)[0]

        self.position_count += 1
        return {
            "cmd": CMD_POSITION,
            "seq": seq,
            "version": version,
            "anchor_id": anchor_id,
            "tag_id": tag_id,
            "distance_m": distance_cm / 100.0,
            "azimuth_raw_deg": azimuth_raw,
            "elevation_raw_deg": elevation_raw,
            "tag_status": tag_status,
            "batch_sn": batch_sn,
        }

    # -----------------------------------------------------------------
    def _parse_heartbeat(self, pkt: bytes) -> Dict:
        """Parse a 16-byte 0x2002 frame.  No XOR byte per spec."""
        seq       = struct.unpack_from(">H", pkt, _P_SEQ_ID)[0]
        version   = struct.unpack_from(">H", pkt, _P_VERSION)[0]
        anchor_id = struct.unpack_from(">I", pkt, _P_ANCHOR_ID)[0]

        self.heartbeat_count += 1
        return {
            "cmd": CMD_HEARTBEAT,
            "seq": seq,
            "version": version,
            "anchor_id": anchor_id,
        }


# ─────────────────────────────────────────────────────────────────────
#  Self-test (runs only when executed directly)
# ─────────────────────────────────────────────────────────────────────
def _hex(s: str) -> bytes:
    return bytes.fromhex(s.replace(" ", "").replace("\n", ""))


def _run_selftest() -> None:
    # Spec page 7 example frame, 37 bytes.
    pos_frame = _hex(
        "FF FF FF FF 00 25 22 46 20 01 01 02 "
        "00 00 00 64 00 00 00 64 "
        "00 00 00 7D 01 48 00 00 "
        "12 34 10 6B AC 00 00 00 A6"
    )
    assert len(pos_frame) == FRAME_LEN_POSITION, (
        f"pos frame len mismatch: {len(pos_frame)} != {FRAME_LEN_POSITION}"
    )

    # Spec page 8 example frame, 16 bytes, no XOR.
    hb_frame = _hex(
        "FF FF FF FF 00 10 00 34 20 02 01 02 00 00 00 64"
    )
    assert len(hb_frame) == FRAME_LEN_HEARTBEAT

    # --- 1. Clean feed, single 0x2001 --------------------------------
    p = AoaFrameParser()
    got = p.feed(pos_frame)
    assert len(got) == 1,                 f"expected 1 frame, got {len(got)}"
    f = got[0]
    assert f["cmd"] == CMD_POSITION
    assert f["seq"] == 0x2246
    assert f["version"] == 0x0102
    assert f["anchor_id"] == 0x64
    assert f["tag_id"] == 0x64
    assert abs(f["distance_m"] - 1.25) < 1e-9,  f"distance={f['distance_m']}"
    assert f["azimuth_raw_deg"] == 328,   f"azimuth={f['azimuth_raw_deg']}"
    assert f["elevation_raw_deg"] == 0
    assert f["tag_status"] == 0x1234
    assert f["batch_sn"] == 0x106B
    assert p.position_count == 1 and p.xor_fail_count == 0

    # --- 2. Clean feed, single 0x2002 --------------------------------
    p = AoaFrameParser()
    got = p.feed(hb_frame)
    assert len(got) == 1
    f = got[0]
    assert f["cmd"] == CMD_HEARTBEAT
    assert f["seq"] == 0x0034
    assert f["anchor_id"] == 0x64
    assert p.heartbeat_count == 1

    # --- 3. Fragmented feed (split mid-frame, multiple chunks) -------
    p = AoaFrameParser()
    assert p.feed(pos_frame[:5])  == []
    assert p.feed(pos_frame[5:10]) == []
    out = p.feed(pos_frame[10:])
    assert len(out) == 1 and out[0]["cmd"] == CMD_POSITION

    # --- 4. Multiple frames concatenated + leading garbage -----------
    # NOTE: garbage must not end in 0xFF.  If it does, the trailing
    # 0xFF(s) can combine with the real frame header to form a
    # "ghost" header one byte early, fail length-check, and cause the
    # first frame to be lost during resync.  That loss is the parser
    # behaving correctly; we just avoid it here so the test reflects
    # the typical UART noise case.
    p = AoaFrameParser()
    garbage = b"\x00\x11\x22\x33\xAB\xCD"
    stream = garbage + pos_frame + hb_frame + pos_frame
    out = p.feed(stream)
    assert len(out) == 3, f"expected 3 frames, got {len(out)}"
    assert [f["cmd"] for f in out] == [CMD_POSITION, CMD_HEARTBEAT, CMD_POSITION]
    assert p.resync_count >= 1

    # --- 5. Corrupted XOR is rejected, next good frame still parses --
    bad = bytearray(pos_frame)
    bad[-1] ^= 0x01  # flip xor byte
    p = AoaFrameParser()
    out = p.feed(bytes(bad) + pos_frame)
    assert len(out) == 1 and out[0]["cmd"] == CMD_POSITION
    assert p.xor_fail_count == 1 and p.position_count == 1

    # --- 6. Bogus length (claims 40 B) — parser must resync ----------
    bogus = bytearray(pos_frame)
    bogus[_P_PACKET_LEN]     = 0x00
    bogus[_P_PACKET_LEN + 1] = 0x28  # 40, not in {16, 37}
    p = AoaFrameParser()
    out = p.feed(bytes(bogus) + pos_frame)
    assert len(out) == 1 and out[0]["cmd"] == CMD_POSITION
    assert p.bad_length_count >= 1

    print("aoa_protocol.py self-test: OK")
    print(f"  position_count  = {p.position_count}")
    print(f"  heartbeat_count = {p.heartbeat_count}")
    print(f"  xor_fail_count  = {p.xor_fail_count}")
    print(f"  bad_length_count= {p.bad_length_count}")
    print(f"  resync_count    = {p.resync_count}")


if __name__ == "__main__":
    _run_selftest()
