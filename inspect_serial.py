"""Raw serial dump: look at exactly what a sensor is sending before trusting
any parser's assumptions about it.

    python inspect_serial.py --port COM5
    python inspect_serial.py --port COM5 --seconds 3 --baud 256000

Prints incoming bytes as hex and as ASCII side by side (classic hexdump
layout), plus a one-line verdict: whether the stream looks like readable
ASCII text or binary data, and if binary, whether it contains either known
sensor's frame header (LD2410C's F4 F3 F2 F1 or LD2450's AA FF 03 00).

This is the first thing to run against any new sensor on this project --
it's how the "is the LD2450 really binary, or is it actually the ASCII
output its spec sheet claims?" question gets settled by looking, not by
guessing. See the note at the top of ld2450.py.
"""

from __future__ import annotations

import argparse
import sys
import time

import ld2410
import ld2450
from serial_backend import DEFAULT_BAUD, SerialLinkError, open_link

ROW_WIDTH = 16


def hexdump(data: bytes, start_offset: int = 0) -> None:
    for row_start in range(0, len(data), ROW_WIDTH):
        row = data[row_start:row_start + ROW_WIDTH]
        hex_part = " ".join("%02X" % b for b in row)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        print("%08X  %-*s  %s" % (start_offset + row_start, ROW_WIDTH * 3 - 1, hex_part, ascii_part))


def verdict(data: bytes) -> str:
    if not data:
        return "no data received -- check wiring (TX/RX not swapped?), baud rate, and power"

    printable = sum(1 for b in data if 32 <= b < 127 or b in (9, 10, 13))
    printable_ratio = printable / len(data)

    has_ld2410_header = ld2410.FRAME_HEADER in data
    has_ld2450_header = ld2450.FRAME_HEADER in data

    lines = []
    if has_ld2410_header:
        lines.append("contains the LD2410C frame header (F4 F3 F2 F1) -- looks like LD2410C binary frames")
    if has_ld2450_header:
        lines.append("contains the LD2450 frame header (AA FF 03 00) -- matches ld2450.py's binary parser")
    if lines:
        return "; ".join(lines)

    if printable_ratio > 0.9:
        return (
            "%.0f%% printable ASCII, no known binary frame header found -- "
            "this looks like TEXT output, not the binary frame ld2450.py currently "
            "expects. Paste a sample of the printed output back and the parser will "
            "need rewriting as a text/line parser instead." % (printable_ratio * 100)
        )
    return (
        "binary data, but neither known frame header (F4 F3 F2 F1 / AA FF 03 00) was "
        "found in this sample -- try a longer --seconds, double-check --baud, or this "
        "may be a different protocol/mode than either parser assumes"
    )


def _main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--seconds", type=float, default=5.0, help="how long to capture")
    args = ap.parse_args()

    try:
        link = open_link(port=args.port, baud=args.baud)
    except SerialLinkError as exc:
        print("error: %s" % exc, file=sys.stderr)
        sys.exit(1)

    print("listening on %s for %.0fs ..." % (link.description, args.seconds))
    captured = bytearray()
    deadline = time.time() + args.seconds
    try:
        while time.time() < deadline:
            chunk = link.read(256)
            if chunk:
                captured.extend(chunk)
    finally:
        link.close()

    print("\n%d bytes captured\n" % len(captured))
    if captured:
        hexdump(bytes(captured[:512]))
        if len(captured) > 512:
            print("... (%d more bytes not shown)" % (len(captured) - 512))
    print("\nverdict: %s" % verdict(bytes(captured)))


if __name__ == "__main__":
    _main()
