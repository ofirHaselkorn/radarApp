"""Work out which sensor is on the other end of a serial link from its bytes.

The LD2410C and the LD2450 share the same UART defaults (256000 8N1) and the
same CH340 adapters, so the port alone says nothing about which one is
wired up. Their report frames don't overlap at all, though: the LD2410C
frames start F4 F3 F2 F1, the LD2450 frames start AA FF 03 00. So rather
than make the user pick, AutoFrameParser feeds every byte to both parsers
until one of them produces a complete, well-formed frame, then locks onto
that parser for the rest of the connection.

It has the same public surface as ld2410.FrameParser (feed/reset/
frames_parsed/frames_rejected/bytes_discarded), so LD2410Reader drives it
through its parser_factory parameter like any other parser.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import ld2410
import ld2450

SENSOR_LD2410C = "LD2410C"
SENSOR_LD2450 = "LD2450"

# Horizontal half-angle of each sensor's useful field of view, in degrees.
# The LD2410C is one fixed beam with no angle output at all (0 = draw a
# single bearing line); the LD2450 reports targets across roughly +/-60 deg.
FIELD_OF_VIEW_HALF_DEG = {SENSOR_LD2410C: 0.0, SENSOR_LD2450: 60.0}


class AutoFrameParser:
    """Sniffs the stream with every known parser until one locks on."""

    def __init__(self, max_buffer: int = 4096):
        self._parsers: Dict[str, object] = {
            SENSOR_LD2410C: ld2410.FrameParser(max_buffer=max_buffer),
            SENSOR_LD2450: ld2450.FrameParser(max_buffer=max_buffer),
        }
        # None until the first good frame; then the sensor kind it came from.
        self.kind: Optional[str] = None

    def reset(self) -> None:
        # A reconnect may well be a different sensor plugged into the same
        # adapter, so forget the lock and sniff again.
        for parser in self._parsers.values():
            parser.reset()
        self.kind = None

    def feed(self, data: bytes) -> List[object]:
        if self.kind is not None:
            return self._parsers[self.kind].feed(data)
        for kind, parser in self._parsers.items():
            frames = parser.feed(data)
            if frames:
                self.kind = kind
                for other_kind, other in self._parsers.items():
                    if other_kind != kind:
                        other.reset()
                return frames
        return []

    # -- counters: the locked parser's, or the worst case while sniffing --
    def _counter(self, name: str) -> int:
        if self.kind is not None:
            return getattr(self._parsers[self.kind], name)
        # While nothing has locked on, every byte is junk to at least one
        # parser; reporting the smaller count would hide a wrong baud rate.
        return max(getattr(p, name) for p in self._parsers.values())

    @property
    def frames_parsed(self) -> int:
        return self._counter("frames_parsed")

    @property
    def frames_rejected(self) -> int:
        return self._counter("frames_rejected")

    @property
    def bytes_discarded(self) -> int:
        return self._counter("bytes_discarded")
