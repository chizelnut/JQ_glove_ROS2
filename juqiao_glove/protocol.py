"""Stream parser for the Juqiao tactile glove serial protocol.

The glove emits two packet types in alternation, both prefixed with a 4-byte
sync `aa 55 03 99`. The first packet (order byte 0x01) carries the first 128
of 256 sensor slots; the second packet (order byte 0x02) carries the remaining
128 sensor slots followed by a 16-byte quaternion. See `layout.py` and the
vendor spec for full details.

The reader is cadence-locked: it identifies a valid frame boundary by checking
that the next-expected sync appears at exactly +134 or +150 bytes. If cadence
breaks (e.g. mid-stream USB enumeration), it resyncs to the next aligned sync
and resumes. This was important for our captures because a naive `data.find(SYNC)`
gives false positives when a real pressure byte happens to be 0xAA followed by
plausible bytes.

The parser is purely a synchronous byte-stream consumer; threading or serial
I/O is the caller's responsibility (see `glove_node.py`).
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Optional
import struct

from .layout import (
    SYNC,
    PACKET_LEN_FIRST,
    PACKET_LEN_SECOND,
    PRESSURE_BYTES_PER_PACKET,
    QUAT_BYTES,
    TOTAL_SENSOR_SLOTS,
    HEX_SIDE,
)

_LEN_BY_ORDER = {0x01: PACKET_LEN_FIRST, 0x02: PACKET_LEN_SECOND}


@dataclass
class GloveSample:
    """One fully-assembled sample from a single glove."""
    sensor_type: int                 # 0x01=LH, 0x02=RH, 0x03=LF, 0x04=RF, 0x05=WB
    side: str                        # 'left' or 'right' (or 'unknown')
    pressure: bytes                  # 256 uint8 values, indices 0..255
    quat_wxyz: tuple                 # (w, x, y, z) float32 in WXYZ order


class GloveStreamParser:
    """Stateful byte-stream parser. Feed bytes via `feed()`, get assembled
    samples via the `on_sample` callback."""

    def __init__(self, on_sample: Callable[[GloveSample], None]):
        self._buf = bytearray()
        self._on_sample = on_sample
        # Partial-sample state: holds the most recently received 0x01 packet's
        # pressure bytes until the matching 0x02 packet arrives.
        self._pending_first: Optional[bytes] = None
        self._pending_first_type: Optional[int] = None

    # ------ public ------------------------------------------------------

    def feed(self, chunk: bytes) -> None:
        """Append bytes from the serial port. Triggers `on_sample(...)` zero
        or more times depending on how many complete samples are formed."""
        if not chunk:
            return
        self._buf.extend(chunk)
        self._drain()

    def reset(self) -> None:
        self._buf.clear()
        self._pending_first = None
        self._pending_first_type = None

    # ------ internal ----------------------------------------------------

    def _is_aligned(self, pos: int) -> bool:
        """A sync at `pos` is aligned iff the frame starting here ends at
        another sync (i.e. its length matches one of the expected lengths)."""
        if pos + 6 > len(self._buf):
            return False
        if self._buf[pos:pos+4] != SYNC:
            return False
        order = self._buf[pos+4]
        flen = _LEN_BY_ORDER.get(order)
        if flen is None:
            return False
        nxt = pos + flen
        if nxt + 4 > len(self._buf):
            return False
        return self._buf[nxt:nxt+4] == SYNC

    def _find_next_aligned(self, start: int) -> Optional[int]:
        """Scan forward for the next cadence-aligned sync. Returns None if
        not enough data yet."""
        pos = start
        while True:
            j = self._buf.find(SYNC, pos)
            if j < 0:
                return None
            if self._is_aligned(j):
                return j
            pos = j + 1

    def _drain(self) -> None:
        """Consume as many complete frames as we have buffered."""
        while True:
            j = self._find_next_aligned(0)
            if j is None:
                # No alignment is decidable yet -- we may simply need more
                # bytes to verify the next sync. Keep buffer intact.
                return
            # Drop everything before j (resync / discard partial garbage).
            if j > 0:
                del self._buf[:j]
            # Now sync is at offset 0. Parse one frame.
            if len(self._buf) < 6:
                return
            order = self._buf[4]
            sensor_type = self._buf[5]
            flen = _LEN_BY_ORDER[order]
            if len(self._buf) < flen:
                return  # need more bytes
            frame = bytes(self._buf[:flen])
            del self._buf[:flen]
            self._handle_frame(order, sensor_type, frame)

    def _handle_frame(self, order: int, sensor_type: int, frame: bytes) -> None:
        if order == 0x01:
            # Cache the first packet's 128 sensor bytes (after 6-byte header).
            self._pending_first = frame[6:6 + PRESSURE_BYTES_PER_PACKET]
            self._pending_first_type = sensor_type
            return
        # order == 0x02
        if (self._pending_first is None
                or self._pending_first_type != sensor_type):
            # No matching first packet (startup or resync). Discard.
            self._pending_first = None
            self._pending_first_type = None
            return
        second_pressure = frame[6:6 + PRESSURE_BYTES_PER_PACKET]
        quat_bytes = frame[
            6 + PRESSURE_BYTES_PER_PACKET:
            6 + PRESSURE_BYTES_PER_PACKET + QUAT_BYTES
        ]
        w, x, y, z = struct.unpack("<4f", quat_bytes)
        full_pressure = self._pending_first + second_pressure
        assert len(full_pressure) == TOTAL_SENSOR_SLOTS, (
            f"expected {TOTAL_SENSOR_SLOTS} bytes, got {len(full_pressure)}"
        )
        self._pending_first = None
        self._pending_first_type = None
        sample = GloveSample(
            sensor_type=sensor_type,
            side=HEX_SIDE.get(sensor_type, "unknown"),
            pressure=full_pressure,
            quat_wxyz=(w, x, y, z),
        )
        try:
            self._on_sample(sample)
        except Exception:  # noqa: BLE001
            # Caller's bug shouldn't kill the read loop. Re-raise after a
            # warning would also be reasonable, but silently dropping a
            # sample is safer for hardware streaming.
            import traceback
            traceback.print_exc()
