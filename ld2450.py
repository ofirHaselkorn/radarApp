"""HLK-LD2450 24GHz multi-target trajectory radar: frame parsing.

Unlike the LD2410C (distance + motion state only, single fixed beam), the
LD2450 has multiple receive antennas and reports real X/Y position for up to
3 simultaneous targets, plus per-target speed -- true distance AND true
angle from one unit, not a configured bearing.

Serial link settings (same UART defaults as the LD2410 family):
    256000 baud, 8 data bits, no parity, 1 stop bit.

CONFIDENCE NOTE: this parser is built from the widely-published community
reverse-engineering of the LD2450 report frame (the same frame layout used
by ESPHome's ld2450 component and several open-source Arduino/Python
libraries), not from a datasheet pasted into this project the way the
LD2410C spec was at the start. The sync bytes and the three-targets-per-
frame layout are well established and safe to rely on. The one detail worth
a real-hardware check is the sign convention on X/Y/speed -- see
verify_sign_convention() at the bottom of this file.

Real hardware (2026-10-22): a live LD2450 on a CH340 at 256000 baud streams
frames that parse cleanly here, with plausible positions -- so the layout
and the Y sign are confirmed. The left/right (X) sign is still to be
checked with a person standing clearly to one side.

Target report frame, 30 bytes total:

    offset  bytes          meaning
    0-3     AA FF 03 00    frame header
    4-11    target 1       X(i16 LE) Y(i16 LE) Speed(i16 LE) Resolution(u16 LE)
    12-19   target 2       same layout
    20-27   target 3       same layout
    28-29   55 CC          frame tail

X, Y are millimetres from the sensor; Y is "forward" (out of the sensor
face), X is lateral. Speed is cm/s. Resolution is the distance-gate
resolution in mm (not a target coordinate) and is currently decoded but
unused. A target slot with all three of X, Y, speed zero means "no target
in that slot" -- the sensor always reports 3 slots, zero-filling the unused
ones, the same spirit as the LD2410C always sending a fixed-size frame.

X/Y/Speed use sign-magnitude, not two's complement: the documented
convention is that the MSB (bit 15) SET means positive and CLEAR means
negative -- backwards from ordinary sign-bit intuition. That is the one
detail this module cannot verify without real hardware in hand.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

FRAME_HEADER = b"\xaa\xff\x03\x00"
FRAME_TAIL = b"\x55\xcc"

TARGET_COUNT = 3
TARGET_STRUCT_LEN = 8  # X, Y, Speed, Resolution: four little-endian int16 fields
FRAME_LEN = len(FRAME_HEADER) + TARGET_COUNT * TARGET_STRUCT_LEN + len(FRAME_TAIL)  # 30

MAGNITUDE_MASK = 0x7FFF
SIGN_BIT = 0x8000


def _signed_magnitude16(raw: int) -> int:
    """Decode the LD2450's sign-magnitude int16: MSB set = positive."""
    magnitude = raw & MAGNITUDE_MASK
    return magnitude if raw & SIGN_BIT else -magnitude


def _encode_signed_magnitude16(value: int) -> int:
    magnitude = min(abs(int(value)), MAGNITUDE_MASK)
    return magnitude | (SIGN_BIT if value >= 0 else 0)


@dataclass(frozen=True)
class Target:
    """One of up to 3 targets in a report frame. present=False means this
    slot was all-zero (the sensor has nothing to report here)."""

    x_mm: int
    y_mm: int
    speed_cms: int
    resolution_mm: int

    @property
    def present(self) -> bool:
        return not (self.x_mm == 0 and self.y_mm == 0 and self.speed_cms == 0)

    @property
    def distance_mm(self) -> float:
        return math.hypot(self.x_mm, self.y_mm)

    @property
    def distance_cm(self) -> float:
        return self.distance_mm / 10.0

    @property
    def bearing_deg(self) -> float:
        """0 = straight ahead, clockwise positive -- the same convention
        radar_scope.py uses for a configured bearing, except this one is
        computed from a real reported position, not configured."""
        return math.degrees(math.atan2(self.x_mm, self.y_mm))


@dataclass(frozen=True)
class TrajectoryFrame:
    """One decoded LD2450 report frame: up to 3 targets."""

    targets: Tuple[Target, Target, Target]
    timestamp: float = 0.0

    @property
    def active_targets(self) -> List[Target]:
        return [t for t in self.targets if t.present]

    @property
    def has_target(self) -> bool:
        return any(t.present for t in self.targets)

    def __str__(self) -> str:
        active = self.active_targets
        if not active:
            return "CLEAR"
        return "  ".join(
            "T%d %5.0fcm @ %+4.0fdeg  %+4dcm/s" % (i + 1, t.distance_cm, t.bearing_deg, t.speed_cms)
            for i, t in enumerate(active)
        )


def _parse_target(buf, offset: int) -> Target:
    x_raw = buf[offset] | (buf[offset + 1] << 8)
    y_raw = buf[offset + 2] | (buf[offset + 3] << 8)
    speed_raw = buf[offset + 4] | (buf[offset + 5] << 8)
    resolution = buf[offset + 6] | (buf[offset + 7] << 8)
    return Target(
        x_mm=_signed_magnitude16(x_raw),
        y_mm=_signed_magnitude16(y_raw),
        speed_cms=_signed_magnitude16(speed_raw),
        resolution_mm=resolution,
    )


def parse_frame_body(body, timestamp: Optional[float] = None) -> Optional[TrajectoryFrame]:
    """Decode the 24 target-data bytes between header and tail."""
    if len(body) != TARGET_COUNT * TARGET_STRUCT_LEN:
        return None
    targets = tuple(_parse_target(body, i * TARGET_STRUCT_LEN) for i in range(TARGET_COUNT))
    return TrajectoryFrame(targets=targets, timestamp=time.time() if timestamp is None else timestamp)


def build_report_frame(targets) -> bytes:
    """Build a 30 byte report frame from up to 3 (x_mm, y_mm, speed_cms)
    tuples (missing slots are zero-filled). Used by the mock link and tests
    -- this is also what makes the parser testable without hardware: encode
    with the same sign convention the decoder assumes, then decode and check
    you get the original values back."""
    body = bytearray()
    targets = list(targets) + [(0, 0, 0)] * (TARGET_COUNT - len(targets))
    for x_mm, y_mm, speed_cms in targets[:TARGET_COUNT]:
        for value in (x_mm, y_mm, speed_cms):
            body += _encode_signed_magnitude16(value).to_bytes(2, "little")
        body += (0).to_bytes(2, "little")  # resolution, unused by the mock
    return FRAME_HEADER + bytes(body) + FRAME_TAIL


class FrameParser:
    """Incremental parser for the LD2450 serial stream.

    Same public surface as ld2410.FrameParser (feed/reset/frames_parsed/
    frames_rejected/bytes_discarded), so LD2410Reader can drive either one
    via its parser_factory parameter once this is wired into main.py.
    """

    def __init__(self, max_buffer: int = 4096):
        self._buf = bytearray()
        self._max_buffer = max_buffer
        self.frames_parsed = 0
        self.frames_rejected = 0
        self.bytes_discarded = 0

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> List[TrajectoryFrame]:
        if not data:
            return []
        self._buf.extend(data)
        if len(self._buf) > self._max_buffer:
            drop = len(self._buf) - self._max_buffer
            del self._buf[:drop]
            self.bytes_discarded += drop

        frames: List[TrajectoryFrame] = []
        now = time.time()

        while True:
            start = self._buf.find(FRAME_HEADER)
            if start < 0:
                keep = len(FRAME_HEADER) - 1
                if len(self._buf) > keep:
                    self.bytes_discarded += len(self._buf) - keep
                    del self._buf[: len(self._buf) - keep]
                break
            if start > 0:
                self.bytes_discarded += start
                del self._buf[:start]

            if len(self._buf) < FRAME_LEN:
                break  # frame still arriving

            if bytes(self._buf[FRAME_LEN - len(FRAME_TAIL): FRAME_LEN]) != FRAME_TAIL:
                self.frames_rejected += 1
                self.bytes_discarded += 1
                del self._buf[:1]
                continue

            body = bytes(self._buf[len(FRAME_HEADER): FRAME_LEN - len(FRAME_TAIL)])
            del self._buf[:FRAME_LEN]

            frame = parse_frame_body(body, timestamp=now)
            if frame is None:
                self.frames_rejected += 1
            else:
                self.frames_parsed += 1
                frames.append(frame)

        return frames


# Display smoothing. Through a door or wall the reported position jitters by
# tens of cm frame to frame and a weak target drops out for a few frames at
# a time; these turn that into a steady dot that stays put briefly when lost.
SMOOTHING_ALPHA = 0.35      # weight of each new reading (~0.25 s lag at ~11 Hz)
HOLD_SECONDS = 1.0          # keep showing a target this long after it drops out
JUMP_RESET_MM = 800         # a bigger jump is a different person in that slot


@dataclass(frozen=True)
class SmoothedTarget:
    """A display position for one target slot. held=True means the sensor
    stopped reporting it and this is its last known position."""

    slot: int
    x_mm: float
    y_mm: float
    speed_cms: int
    last_seen: float
    held: bool = False

    @property
    def distance_cm(self) -> float:
        return math.hypot(self.x_mm, self.y_mm) / 10.0

    @property
    def bearing_deg(self) -> float:
        return math.degrees(math.atan2(self.x_mm, self.y_mm))


class TargetSmoother:
    """Per-slot exponential smoothing plus a short hold, for display only --
    raw frames (and the field log) are untouched. Feed each new frame once
    with update(); read targets(now) as often as the UI redraws."""

    def __init__(self, alpha: float = SMOOTHING_ALPHA, hold_seconds: float = HOLD_SECONDS,
                 jump_reset_mm: float = JUMP_RESET_MM):
        self.alpha = alpha
        self.hold_seconds = hold_seconds
        self.jump_reset_mm = jump_reset_mm
        self._slots: Dict[int, SmoothedTarget] = {}
        self._latest = 0.0

    def reset(self) -> None:
        self._slots.clear()
        self._latest = 0.0

    def update(self, frame: TrajectoryFrame) -> None:
        self._latest = frame.timestamp
        for index, target in enumerate(frame.targets):
            if not target.present:
                continue
            slot = index + 1
            prev = self._slots.get(slot)
            if prev is None or math.hypot(target.x_mm - prev.x_mm, target.y_mm - prev.y_mm) > self.jump_reset_mm:
                x, y = float(target.x_mm), float(target.y_mm)
            else:
                x = prev.x_mm + self.alpha * (target.x_mm - prev.x_mm)
                y = prev.y_mm + self.alpha * (target.y_mm - prev.y_mm)
            self._slots[slot] = SmoothedTarget(slot, x, y, target.speed_cms, frame.timestamp)

    def targets(self, now: float) -> List[SmoothedTarget]:
        out = []
        for slot in sorted(self._slots):
            tracked = self._slots[slot]
            if now - tracked.last_seen > self.hold_seconds:
                del self._slots[slot]
                continue
            held = tracked.last_seen < self._latest
            out.append(SmoothedTarget(slot, tracked.x_mm, tracked.y_mm,
                                      0 if held else tracked.speed_cms, tracked.last_seen, held))
        return out


def verify_sign_convention(x_mm: int, known_direction: str) -> bool:
    """Run this once with real hardware, not before: stand facing the
    sensor's front, have a target walk to one side of it, read the X value
    off a live frame, and check it here.

        >>> from ld2450 import verify_sign_convention
        >>> verify_sign_convention(x_mm=-842, known_direction="left")
        True

    known_direction is "left" or "right" as someone standing BEHIND the
    sensor, facing the same way it is, would call it (i.e. the sensor's own
    left/right, not the target's). A True result confirms this module's
    sign convention matches reality. A False result on an unambiguous
    one-sided reading means X (and by the same convention, Y and speed) are
    mirrored from what _signed_magnitude16 assumes -- flip the sign-bit test
    there (SET=negative instead of SET=positive) and re-run this check.
    """
    if known_direction == "left":
        return x_mm < 0
    if known_direction == "right":
        return x_mm > 0
    raise ValueError('known_direction must be "left" or "right"')
