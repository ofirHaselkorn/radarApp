"""Sensor auto-detection tests. Run with: python test_sensor_detect.py

Checks that AutoFrameParser locks onto whichever sensor's frames actually
arrive, ignores leading junk, re-sniffs after reset() (a different sensor
on the same adapter), and that both mock links are detected as the sensor
they pretend to be.
"""

import ld2410
import ld2450
from sensor_detect import SENSOR_LD2410C, SENSOR_LD2450, AutoFrameParser
from serial_backend import MockLD2450Link, MockLink

PASSED = []

# A real frame captured from the user's LD2450 on 2026-10-22.
REAL_LD2450_FRAME = bytes.fromhex(
    "aaff03005f00f2800000680100000000000000000000000000000000" "55cc"
)


def check(label, condition):
    assert condition, "FAILED: " + label
    PASSED.append(label)


def test_starts_undetected():
    check("kind is None before any data", AutoFrameParser().kind is None)


def test_detects_ld2410():
    parser = AutoFrameParser()
    frames = parser.feed(ld2410.build_basic_frame(ld2410.STATE_MOVING, moving_distance_cm=150))
    check("ld2410 frame returned", len(frames) == 1)
    check("locked onto LD2410C", parser.kind == SENSOR_LD2410C)
    check("ld2410 frame type", isinstance(frames[0], ld2410.TargetFrame))


def test_detects_ld2450():
    parser = AutoFrameParser()
    frames = parser.feed(ld2450.build_report_frame([(300, 1200, 0)]))
    check("ld2450 frame returned", len(frames) == 1)
    check("locked onto LD2450", parser.kind == SENSOR_LD2450)
    check("ld2450 frame type", isinstance(frames[0], ld2450.TrajectoryFrame))


def test_real_ld2450_capture():
    parser = AutoFrameParser()
    frames = parser.feed(b"\x00\x13\x88" + REAL_LD2450_FRAME * 2)
    check("real capture detected as LD2450", parser.kind == SENSOR_LD2450)
    check("both real frames decoded", len(frames) == 2)
    target = frames[0].active_targets[0]
    check("real capture y is 242 mm in front", target.y_mm == 242)
    check("real capture x magnitude is 95 mm", abs(target.x_mm) == 95)


def test_byte_split_detection():
    parser = AutoFrameParser()
    frame = ld2450.build_report_frame([(-500, 2000, 12)])
    got = []
    for b in frame:
        got += parser.feed(bytes([b]))
    check("byte-at-a-time stream still detected", parser.kind == SENSOR_LD2450 and len(got) == 1)


def test_stays_locked():
    parser = AutoFrameParser()
    parser.feed(ld2450.build_report_frame([(0, 1000, 0)]))
    frames = parser.feed(ld2410.build_basic_frame(ld2410.STATE_MOVING, moving_distance_cm=150))
    check("locked parser ignores the other sensor's frames", frames == [] and parser.kind == SENSOR_LD2450)


def test_reset_resniffs():
    parser = AutoFrameParser()
    parser.feed(ld2450.build_report_frame([(0, 1000, 0)]))
    parser.reset()
    check("reset forgets the lock", parser.kind is None)
    parser.feed(ld2410.build_basic_frame(ld2410.STATE_STATIONARY, stationary_distance_cm=90))
    check("re-detects a different sensor after reset", parser.kind == SENSOR_LD2410C)


def test_counters_follow_locked_parser():
    parser = AutoFrameParser()
    parser.feed(ld2450.build_report_frame([(0, 1000, 0)]) * 3)
    check("frames_parsed counts the locked parser", parser.frames_parsed == 3)


def test_junk_counted_while_sniffing():
    parser = AutoFrameParser()
    parser.feed(bytes(range(200)))
    check("junk shows up before any lock", parser.kind is None and parser.bytes_discarded > 0)


def _sniff_link(link, attempts=40):
    parser = AutoFrameParser()
    for _ in range(attempts):
        if parser.feed(link.read()):
            return parser.kind
    return None


def test_mock_links_detected():
    check("MockLink detected as LD2410C", _sniff_link(MockLink()) == SENSOR_LD2410C)
    check("MockLD2450Link detected as LD2450", _sniff_link(MockLD2450Link()) == SENSOR_LD2450)


if __name__ == "__main__":
    for name, fn in sorted(list(globals().items())):
        if name.startswith("test_"):
            fn()
    for label in PASSED:
        print("  ok  " + label)
    print("\n%d checks passed" % len(PASSED))
