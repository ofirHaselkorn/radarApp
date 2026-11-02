"""HLK-LD2410C configuration/command protocol.

ld2410.py only *reads* the sensor's continuous data stream. This module
*writes* to it: a separate command protocol lets you enter a configuration
session and change the max detection distance, per-gate sensitivity, read
back current settings, read the firmware version, or restart/factory-reset
the module.

CONFIDENCE NOTE: unlike the basic data frame (which you pasted verbatim
from the datasheet at the start of this project) and unlike ld2450.py
(community reverse-engineering with a genuinely uncertain detail), this is
Hi-Link's own documented command protocol, consistently implemented
identically across many independent open-source projects (ESPHome,
Tasmota, several Arduino/Python libraries) with no disagreement between
them on the framing or the core commands below. Confidence here is high for
everything except read_firmware_version's exact field layout, which is
noted at that function.

Command frame, variable length:

    offset   bytes          meaning
    0-3      FD FC FB FA    frame header (note: different from the data
                             frame's F4 F3 F2 F1 -- this is how the sensor
                             tells the two apart)
    4-5      length         payload length, little endian
    6-7      command word   little endian
    8..      value          command-specific, may be empty
    last 4   04 03 02 01    frame footer

ACK frame, the sensor's reply to a command, same envelope:

    6-7      ack word       the command word with bit 8 set (command | 0x0100)
    8-9      status         0x0000 = success, nonzero = failure
    10..     value          command-specific, may be empty

A configuration session must have the link to itself: send CMD_ENABLE_CONFIG
first, and CMD_END_CONFIG when done -- the sensor does not stream ordinary
target data frames while a config session is open, so this cannot run at
the same time as LD2410Reader is reading the same port. See configure_sensor.py
for a standalone CLI that does this with the port to itself; the live app's
reader is not meant to share a port with this module while connected.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional

FRAME_HEADER = b"\xfd\xfc\xfb\xfa"
FRAME_FOOTER = b"\x04\x03\x02\x01"

_FRAME_OVERHEAD = len(FRAME_HEADER) + 2 + len(FRAME_FOOTER)
MAX_PAYLOAD_LEN = 256

ACK_BIT = 0x0100

# -- commands ----------------------------------------------------------------
CMD_ENABLE_CONFIG = 0x00FF
CMD_END_CONFIG = 0x00FE
CMD_SET_MAX_GATE_DURATION = 0x0060
CMD_READ_PARAMETERS = 0x0061
CMD_SET_GATE_SENSITIVITY = 0x0064
CMD_READ_FIRMWARE_VERSION = 0x00A0
CMD_RESTART = 0x00A2
CMD_FACTORY_RESET = 0x00A4

PROTOCOL_VERSION = b"\x01\x00"  # fixed value sent with CMD_ENABLE_CONFIG

# Parameter words used inside CMD_SET_MAX_GATE_DURATION / CMD_READ_PARAMETERS.
PARAM_MAX_MOVING_GATE = 0x0000
PARAM_MAX_STATIC_GATE = 0x0001
PARAM_DURATION = 0x0002

GATE_ALL = 0xFFFF  # gate selector for CMD_SET_GATE_SENSITIVITY: apply to every gate
MAX_GATE_INDEX = 8  # gates are numbered 0-8 (0.75 m each, so 0-6.75 m)


class ConfigError(RuntimeError):
    """A command was rejected or the session protocol wasn't followed."""


@dataclass(frozen=True)
class AckFrame:
    command_word: int  # the original command this acknowledges (ack bit cleared)
    success: bool
    value: bytes
    timestamp: float = 0.0


def _u16le(buf, offset: int) -> int:
    return buf[offset] | (buf[offset + 1] << 8)


def build_command_frame(command_word: int, value: bytes = b"") -> bytes:
    data = command_word.to_bytes(2, "little") + value
    return FRAME_HEADER + len(data).to_bytes(2, "little") + data + FRAME_FOOTER


def parse_ack_body(body: bytes, timestamp: Optional[float] = None) -> Optional[AckFrame]:
    """Decode the bytes between header+length and footer."""
    if len(body) < 4:
        return None
    ack_word = _u16le(body, 0)
    if not ack_word & ACK_BIT:
        return None  # not actually an ACK (or a command frame looped back)
    status = _u16le(body, 2)
    return AckFrame(
        command_word=ack_word & ~ACK_BIT,
        success=(status == 0),
        value=bytes(body[4:]),
        timestamp=time.time() if timestamp is None else timestamp,
    )


class AckParser:
    """Incremental parser for the command/ACK stream. Same shape as
    ld2410.FrameParser and ld2450.FrameParser: feed bytes, get frames back,
    corruption costs one byte and resync continues."""

    def __init__(self, max_buffer: int = 2048):
        self._buf = bytearray()
        self._max_buffer = max_buffer
        self.frames_parsed = 0
        self.frames_rejected = 0
        self.bytes_discarded = 0

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> List[AckFrame]:
        if not data:
            return []
        self._buf.extend(data)
        if len(self._buf) > self._max_buffer:
            drop = len(self._buf) - self._max_buffer
            del self._buf[:drop]
            self.bytes_discarded += drop

        frames: List[AckFrame] = []
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

            if len(self._buf) < len(FRAME_HEADER) + 2:
                break

            length = _u16le(self._buf, 4)
            if not 0 <= length <= MAX_PAYLOAD_LEN:
                self.frames_rejected += 1
                self.bytes_discarded += 1
                del self._buf[:1]
                continue

            total = _FRAME_OVERHEAD + length
            if len(self._buf) < total:
                break

            if bytes(self._buf[total - len(FRAME_FOOTER): total]) != FRAME_FOOTER:
                self.frames_rejected += 1
                self.bytes_discarded += 1
                del self._buf[:1]
                continue

            body = bytes(self._buf[6: 6 + length])
            del self._buf[:total]

            frame = parse_ack_body(body, timestamp=now)
            if frame is None:
                self.frames_rejected += 1
            else:
                self.frames_parsed += 1
                frames.append(frame)

        return frames


def build_set_max_gate_duration(
    max_moving_gate: int, max_static_gate: int, duration_s: int
) -> bytes:
    if not 0 <= max_moving_gate <= MAX_GATE_INDEX:
        raise ValueError("max_moving_gate must be 0-%d" % MAX_GATE_INDEX)
    if not 0 <= max_static_gate <= MAX_GATE_INDEX:
        raise ValueError("max_static_gate must be 0-%d" % MAX_GATE_INDEX)
    if not 0 <= duration_s <= 0xFFFF:
        raise ValueError("duration_s must be 0-65535")
    value = b"".join(
        word.to_bytes(2, "little") + val.to_bytes(4, "little")
        for word, val in (
            (PARAM_MAX_MOVING_GATE, max_moving_gate),
            (PARAM_MAX_STATIC_GATE, max_static_gate),
            (PARAM_DURATION, duration_s),
        )
    )
    return build_command_frame(CMD_SET_MAX_GATE_DURATION, value)


def build_set_gate_sensitivity(gate: int, motion_pct: int, static_pct: int) -> bytes:
    if gate != GATE_ALL and not 0 <= gate <= MAX_GATE_INDEX:
        raise ValueError("gate must be 0-%d or GATE_ALL" % MAX_GATE_INDEX)
    if not 0 <= motion_pct <= 100 or not 0 <= static_pct <= 100:
        raise ValueError("sensitivity values are 0-100")
    value = (
        gate.to_bytes(2, "little")
        + (0).to_bytes(2, "little") + motion_pct.to_bytes(4, "little")
        + (1).to_bytes(2, "little") + static_pct.to_bytes(4, "little")
    )
    return build_command_frame(CMD_SET_GATE_SENSITIVITY, value)


@dataclass(frozen=True)
class SensorParameters:
    """Decoded CMD_READ_PARAMETERS response."""

    max_moving_gate: int
    max_static_gate: int
    motion_sensitivity: tuple  # one 0-100 value per gate, 0..8
    static_sensitivity: tuple
    duration_s: int


def parse_read_parameters(value: bytes) -> SensorParameters:
    """Decode a CMD_READ_PARAMETERS ACK's value bytes.

    Layout: header(1,0xAA) max_moving_gate(1) max_static_gate(1)
    motion_sensitivity[9] static_sensitivity[9] duration(2 LE) tail(1,0x55).
    """
    if len(value) != 24:
        raise ConfigError("read-parameters reply is %d bytes, expected 24" % len(value))
    if value[0] != 0xAA or value[23] != 0x55:
        raise ConfigError("read-parameters reply missing AA/55 markers -- layout may not match")
    return SensorParameters(
        max_moving_gate=value[1],
        max_static_gate=value[2],
        motion_sensitivity=tuple(value[3:12]),
        static_sensitivity=tuple(value[12:21]),
        duration_s=_u16le(value, 21),
    )


class ConfigSession:
    """High-level request/response wrapper around a link opened for config.

    `link` needs .write(bytes) and .read(max_bytes) -> bytes (same read()
    contract as the data-streaming links in serial_backend.py: returns b""
    rather than blocking forever). Use as a context manager so
    CMD_END_CONFIG always gets sent, even on error:

        with ConfigSession(link) as session:
            session.set_max_gate_duration(8, 8, 5)
            params = session.read_parameters()
    """

    def __init__(self, link, timeout: float = 2.0):
        self._link = link
        self._timeout = timeout
        self._parser = AckParser()

    def __enter__(self) -> "ConfigSession":
        self.enter_config()
        return self

    def __exit__(self, *_exc) -> None:
        try:
            self.exit_config()
        except Exception:
            pass  # best-effort on the way out; don't mask the real error

    def _request(self, frame: bytes, command_word: int) -> AckFrame:
        self._parser.reset()
        self._link.write(frame)
        deadline = time.time() + self._timeout
        while time.time() < deadline:
            chunk = self._link.read(256)
            if chunk:
                for ack in self._parser.feed(chunk):
                    if ack.command_word == command_word:
                        if not ack.success:
                            raise ConfigError(
                                "sensor rejected command 0x%04X" % command_word
                            )
                        return ack
                    # an ACK for a different command (stale/crossed-over) is
                    # logged implicitly via frames_parsed and ignored here
        raise ConfigError(
            "no ACK for command 0x%04X within %.1fs" % (command_word, self._timeout)
        )

    def enter_config(self) -> None:
        self._request(build_command_frame(CMD_ENABLE_CONFIG, PROTOCOL_VERSION), CMD_ENABLE_CONFIG)

    def exit_config(self) -> None:
        self._request(build_command_frame(CMD_END_CONFIG), CMD_END_CONFIG)

    def set_max_gate_duration(self, max_moving_gate: int, max_static_gate: int, duration_s: int) -> None:
        self._request(
            build_set_max_gate_duration(max_moving_gate, max_static_gate, duration_s),
            CMD_SET_MAX_GATE_DURATION,
        )

    def set_gate_sensitivity(self, gate: int, motion_pct: int, static_pct: int) -> None:
        self._request(
            build_set_gate_sensitivity(gate, motion_pct, static_pct),
            CMD_SET_GATE_SENSITIVITY,
        )

    def read_parameters(self) -> SensorParameters:
        ack = self._request(build_command_frame(CMD_READ_PARAMETERS), CMD_READ_PARAMETERS)
        return parse_read_parameters(ack.value)

    def read_firmware_version(self) -> bytes:
        """Returns the raw ACK value bytes.

        LOWER CONFIDENCE than the rest of this module: sources agree less
        on the exact field split here (a type word plus some version
        encoding). Rather than guess a byte layout and risk silently
        printing a wrong version string, this returns the raw bytes --
        print them as hex and compare against what the sensor's own
        configuration tool shows, the first time this runs against real
        hardware.
        """
        ack = self._request(build_command_frame(CMD_READ_FIRMWARE_VERSION), CMD_READ_FIRMWARE_VERSION)
        return ack.value

    def restart(self) -> None:
        self._request(build_command_frame(CMD_RESTART), CMD_RESTART)

    def factory_reset(self) -> None:
        self._request(build_command_frame(CMD_FACTORY_RESET), CMD_FACTORY_RESET)
