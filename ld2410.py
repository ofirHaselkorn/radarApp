"""HLK-LD2410C mmWave radar: frame parsing and background reading.

Serial link settings expected by the sensor (factory default):
    256000 baud, 8 data bits, no parity, 1 stop bit.

Basic Target Data frame, 23 bytes total:

    offset  bytes          meaning
    0-3     F4 F3 F2 F1    frame header
    4-5     0D 00          payload length, little endian (13)
    6       02             data type (02 = basic target data)
    7       AA             payload head
    8       xx             target state (see STATE_* below)
    9-10    lo hi          moving target distance, cm
    11      xx             moving target energy, 0-100
    12-13   lo hi          stationary target distance, cm
    14      xx             stationary target energy, 0-100
    15-16   lo hi          detection distance, cm
    17      55             payload tail
    18      00             check byte (always 00 on this sensor)
    19-22   F8 F7 F6 F5    frame footer

The parser is driven by the length field rather than a hard-coded 23, so
engineering-mode frames (data type 01, longer payload) are accepted too; their
first 11 payload bytes carry the same basic target data.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

FRAME_HEADER = b"\xf4\xf3\xf2\xf1"
FRAME_FOOTER = b"\xf8\xf7\xf6\xf5"

PAYLOAD_HEAD = 0xAA
PAYLOAD_TAIL = 0x55

DATA_TYPE_ENGINEERING = 0x01
DATA_TYPE_BASIC = 0x02

STATE_NO_TARGET = 0x00
STATE_MOVING = 0x01
STATE_STATIONARY = 0x02
STATE_BOTH = 0x03

STATE_NAMES = {
    STATE_NO_TARGET: "NO TARGET",
    STATE_MOVING: "MOVING",
    STATE_STATIONARY: "STATIONARY",
    STATE_BOTH: "MOVING + STATIONARY",
}

# Bytes 6..18 of a basic frame. Anything shorter cannot hold target data.
BASIC_PAYLOAD_LEN = 13
# Sanity bound, so a length field that came from noise is rejected.
MAX_PAYLOAD_LEN = 512

_FRAME_OVERHEAD = len(FRAME_HEADER) + 2 + len(FRAME_FOOTER)


@dataclass(frozen=True)
class TargetFrame:
    """One decoded basic target data frame."""

    state: int
    moving_distance_cm: int
    moving_energy: int
    stationary_distance_cm: int
    stationary_energy: int
    detection_distance_cm: int
    data_type: int = DATA_TYPE_BASIC
    timestamp: float = 0.0

    @property
    def has_moving(self) -> bool:
        return bool(self.state & STATE_MOVING)

    @property
    def has_stationary(self) -> bool:
        return bool(self.state & STATE_STATIONARY)

    @property
    def has_target(self) -> bool:
        return self.state > STATE_NO_TARGET

    @property
    def state_name(self) -> str:
        return STATE_NAMES.get(self.state, "UNKNOWN 0x%02X" % self.state)

    @property
    def distance_cm(self) -> Optional[int]:
        """Distance to report, or None when the room is clear.

        A moving target wins over a stationary one, which is what you want on a
        tactical readout: movement is the thing that matters.
        """
        if self.has_moving:
            return self.moving_distance_cm
        if self.has_stationary:
            return self.stationary_distance_cm
        return None

    @property
    def energy(self) -> Optional[int]:
        """Energy of the target reported by :attr:`distance_cm`."""
        if self.has_moving:
            return self.moving_energy
        if self.has_stationary:
            return self.stationary_energy
        return None

    def __str__(self) -> str:
        if not self.has_target:
            return "CLEAR"
        return "%-19s %4d cm  move %4d cm/%3d  static %4d cm/%3d  detect %4d cm" % (
            self.state_name,
            self.distance_cm,
            self.moving_distance_cm,
            self.moving_energy,
            self.stationary_distance_cm,
            self.stationary_energy,
            self.detection_distance_cm,
        )


def _u16le(buf, offset: int) -> int:
    """Little endian 16 bit read: low byte first, then high byte."""
    return buf[offset] | (buf[offset + 1] << 8)


def parse_payload(payload, timestamp: Optional[float] = None) -> Optional[TargetFrame]:
    """Decode a payload (bytes 6..n of a frame) into a :class:`TargetFrame`.

    Returns None when the payload is not valid basic target data.
    """
    if len(payload) < BASIC_PAYLOAD_LEN:
        return None

    data_type = payload[0]
    if data_type not in (DATA_TYPE_BASIC, DATA_TYPE_ENGINEERING):
        return None
    if payload[1] != PAYLOAD_HEAD:
        return None
    # Basic frames must match the documented layout exactly. Engineering frames
    # carry extra per-gate data between the target block and the tail.
    if data_type == DATA_TYPE_BASIC:
        if len(payload) != BASIC_PAYLOAD_LEN or payload[11] != PAYLOAD_TAIL:
            return None
    elif payload[-2] != PAYLOAD_TAIL:
        return None

    return TargetFrame(
        state=payload[2],
        moving_distance_cm=_u16le(payload, 3),
        moving_energy=payload[5],
        stationary_distance_cm=_u16le(payload, 6),
        stationary_energy=payload[8],
        detection_distance_cm=_u16le(payload, 9),
        data_type=data_type,
        timestamp=time.time() if timestamp is None else timestamp,
    )


def build_basic_frame(
    state: int,
    moving_distance_cm: int = 0,
    moving_energy: int = 0,
    stationary_distance_cm: int = 0,
    stationary_energy: int = 0,
    detection_distance_cm: int = 0,
) -> bytes:
    """Build a 23 byte basic target data frame (used by the mock link and tests)."""
    payload = bytes(
        [
            DATA_TYPE_BASIC,
            PAYLOAD_HEAD,
            state & 0xFF,
            moving_distance_cm & 0xFF,
            (moving_distance_cm >> 8) & 0xFF,
            moving_energy & 0xFF,
            stationary_distance_cm & 0xFF,
            (stationary_distance_cm >> 8) & 0xFF,
            stationary_energy & 0xFF,
            detection_distance_cm & 0xFF,
            (detection_distance_cm >> 8) & 0xFF,
            PAYLOAD_TAIL,
            0x00,
        ]
    )
    return FRAME_HEADER + len(payload).to_bytes(2, "little") + payload + FRAME_FOOTER


class FrameParser:
    """Incremental parser for the LD2410 serial stream.

    Feed it whatever the serial port hands you, in any chunk size; it buffers
    partial frames, resynchronises after corruption, and returns the frames it
    could decode.
    """

    def __init__(self, max_buffer: int = 4096):
        self._buf = bytearray()
        self._max_buffer = max_buffer
        self.frames_parsed = 0
        self.frames_rejected = 0
        self.bytes_discarded = 0

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> List[TargetFrame]:
        """Add raw bytes, return every complete frame they completed."""
        if not data:
            return []

        self._buf.extend(data)
        # Hard cap, so a stream of garbage can never grow the buffer unbounded.
        if len(self._buf) > self._max_buffer:
            drop = len(self._buf) - self._max_buffer
            del self._buf[:drop]
            self.bytes_discarded += drop

        frames: List[TargetFrame] = []
        now = time.time()

        while True:
            start = self._buf.find(FRAME_HEADER)
            if start < 0:
                # Keep the last 3 bytes: a header can straddle two reads.
                keep = len(FRAME_HEADER) - 1
                if len(self._buf) > keep:
                    self.bytes_discarded += len(self._buf) - keep
                    del self._buf[: len(self._buf) - keep]
                break
            if start > 0:
                self.bytes_discarded += start
                del self._buf[:start]

            if len(self._buf) < len(FRAME_HEADER) + 2:
                break  # length field has not arrived yet

            length = _u16le(self._buf, 4)
            if not 1 <= length <= MAX_PAYLOAD_LEN:
                self._resync()
                continue

            total = _FRAME_OVERHEAD + length
            if len(self._buf) < total:
                break  # frame still arriving

            if bytes(self._buf[total - len(FRAME_FOOTER): total]) != FRAME_FOOTER:
                self._resync()
                continue

            payload = bytes(self._buf[6: 6 + length])
            del self._buf[:total]

            frame = parse_payload(payload, timestamp=now)
            if frame is None:
                self.frames_rejected += 1
            else:
                self.frames_parsed += 1
                frames.append(frame)

        return frames

    def _resync(self) -> None:
        """Drop one byte, so the next find() can latch onto a later header."""
        self.frames_rejected += 1
        self.bytes_discarded += 1
        del self._buf[:1]


# Reader states handed to the status callback.
STATUS_CONNECTING = "connecting"
STATUS_CONNECTED = "connected"
STATUS_STALE = "stale"
STATUS_ERROR = "error"
STATUS_STOPPED = "stopped"


class LD2410Reader:
    """Reads a sensor on a background thread and pushes frames to callbacks.

    Nothing here touches a UI toolkit and nothing blocks the caller: start()
    returns immediately. The callbacks run on the reader thread, so a UI has to
    marshal them onto its own thread (see main.py).

    Despite the name, this isn't LD2410-specific: parser_factory defaults to
    ld2410.FrameParser but accepts anything with the same surface (feed/
    reset/frames_parsed/frames_rejected/bytes_discarded) -- ld2450.FrameParser
    is a drop-in, so one reader class drives either sensor's stream.
    """

    def __init__(
        self,
        open_link: Callable[[], object],
        on_frame: Optional[Callable[[object], None]] = None,
        on_status: Optional[Callable[[str, str], None]] = None,
        reconnect_delay: float = 2.0,
        stale_after: float = 1.0,
        read_chunk: int = 256,
        parser_factory: Callable[[], object] = FrameParser,
    ):
        self._open_link = open_link
        self._on_frame = on_frame
        self._on_status = on_status
        self._reconnect_delay = reconnect_delay
        self._stale_after = stale_after
        self._read_chunk = read_chunk
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.parser = parser_factory()

    # -- lifecycle --------------------------------------------------------
    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.is_running:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="ld2410-reader", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout)

    # -- internals --------------------------------------------------------
    def _status(self, state: str, detail: str = "") -> None:
        if self._on_status is None:
            return
        try:
            self._on_status(state, detail)
        except Exception:  # a broken UI callback must not kill the reader
            pass

    def _emit(self, frame: TargetFrame) -> None:
        if self._on_frame is None:
            return
        try:
            self._on_frame(frame)
        except Exception:
            pass

    def _run(self) -> None:
        while not self._stop.is_set():
            self._status(STATUS_CONNECTING, "opening serial port")
            try:
                link = self._open_link()
            except Exception as exc:
                self._status(STATUS_ERROR, str(exc) or exc.__class__.__name__)
                self._stop.wait(self._reconnect_delay)
                continue

            self._status(STATUS_CONNECTED, getattr(link, "description", "serial"))
            self.parser.reset()
            last_data = time.time()
            stale_sent = False

            try:
                while not self._stop.is_set():
                    chunk = link.read(self._read_chunk)
                    if chunk:
                        for frame in self.parser.feed(chunk):
                            last_data = frame.timestamp
                            stale_sent = False
                            self._emit(frame)
                    if not stale_sent and time.time() - last_data > self._stale_after:
                        stale_sent = True
                        self._status(STATUS_STALE, "no frames from sensor")
            except Exception as exc:
                self._status(STATUS_ERROR, str(exc) or exc.__class__.__name__)
                self._stop.wait(self._reconnect_delay)
            finally:
                try:
                    link.close()
                except Exception:
                    pass

        self._status(STATUS_STOPPED, "reader stopped")


def _main() -> None:
    """Terminal smoke test: python ld2410.py [--port COM5] [--mock]"""
    import argparse

    from serial_backend import make_link_factory

    ap = argparse.ArgumentParser(description="Dump LD2410C target frames")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=256000)
    ap.add_argument("--mock", action="store_true", help="use the synthetic sensor")
    args = ap.parse_args()

    reader = LD2410Reader(
        open_link=make_link_factory(port=args.port, baud=args.baud, mock=args.mock),
        on_frame=lambda f: print(f),
        on_status=lambda state, detail: print("[%s] %s" % (state.upper(), detail)),
    )
    reader.start()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()


if __name__ == "__main__":
    _main()
