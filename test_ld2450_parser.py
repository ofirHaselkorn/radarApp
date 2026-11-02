"""LD2450 protocol tests. Run with: python test_ld2450_parser.py

These validate frame sync, multi-target decoding, and the sign-magnitude
round-trip -- everything that can be checked without hardware. They do NOT
and cannot validate that the sign convention matches the real sensor; that
needs ld2450.verify_sign_convention() run against a live reading once the
hardware is in hand (see the note at the top of ld2450.py).
"""

import ld2450
from ld2450 import FrameParser, build_report_frame, parse_frame_body

PASSED = []


def check(label, condition):
    assert condition, "FAILED: " + label
    PASSED.append(label)


def test_frame_length_and_sync_bytes():
    frame = build_report_frame([(100, 200, 0)])
    check("frame is 30 bytes", len(frame) == 30)
    check("starts with AA FF 03 00", frame[:4] == ld2450.FRAME_HEADER)
    check("ends with 55 CC", frame[-2:] == ld2450.FRAME_TAIL)


def test_single_target_round_trip():
    frame = build_report_frame([(842, 1500, -37)])
    decoded = FrameParser().feed(frame)
    check("one frame decoded", len(decoded) == 1)
    targets = decoded[0].active_targets
    check("one active target", len(targets) == 1)
    t = targets[0]
    check("x_mm round-trips", t.x_mm == 842)
    check("y_mm round-trips", t.y_mm == 1500)
    check("speed_cms round-trips", t.speed_cms == -37)
    check("distance_cm matches hypot", abs(t.distance_cm - (842**2 + 1500**2) ** 0.5 / 10.0) < 0.01)


def test_negative_values_round_trip():
    frame = build_report_frame([(-842, 1500, -37), (0, -2000, 15)])
    decoded = FrameParser().feed(frame)[0]
    check("negative x_mm round-trips", decoded.targets[0].x_mm == -842)
    check("negative speed round-trips", decoded.targets[0].speed_cms == -37)
    check("negative y_mm round-trips", decoded.targets[1].y_mm == -2000)


def test_three_targets():
    frame = build_report_frame([(100, 500, 10), (-300, 800, -5), (0, 1200, 0)])
    decoded = FrameParser().feed(frame)[0]
    check("three active targets", len(decoded.active_targets) == 3)
    check("has_target true", decoded.has_target)


def test_empty_slots_not_present():
    # Only one real target; the other two slots stay all-zero.
    frame = build_report_frame([(400, 600, 20)])
    decoded = FrameParser().feed(frame)[0]
    check("one present target out of three slots", len(decoded.active_targets) == 1)
    check("empty slots not present", not decoded.targets[1].present and not decoded.targets[2].present)
    check("empty room (no targets) has_target False",
          not build_zero_frame_decoded().has_target)


def build_zero_frame_decoded():
    return FrameParser().feed(build_report_frame([]))[0]


def test_bearing_sign_matches_x_sign():
    # bearing_deg uses atan2(x, y) with the same "0 = forward, clockwise
    # positive" convention as radar_scope.py's configured bearings. A target
    # dead ahead (x=0) should read ~0 deg; one off to +x should read a
    # positive bearing.
    frame = build_report_frame([(0, 1000, 0), (500, 1000, 0)])
    decoded = FrameParser().feed(frame)[0]
    check("straight-ahead target near 0 deg", abs(decoded.targets[0].bearing_deg) < 1.0)
    check("positive-x target has positive bearing", decoded.targets[1].bearing_deg > 0)


def test_byte_split_stream():
    parser = FrameParser()
    frame = build_report_frame([(123, 456, 7)])
    decoded = []
    for i in range(len(frame)):
        decoded.extend(parser.feed(frame[i:i + 1]))
    check("split stream yields one frame", len(decoded) == 1)
    check("split stream x_mm correct", decoded[0].targets[0].x_mm == 123)


def test_back_to_back_frames():
    parser = FrameParser()
    stream = b"".join(build_report_frame([(d, 1000, 0)]) for d in (50, 150, 250))
    frames = parser.feed(stream)
    check("three frames in one read", len(frames) == 3)
    check("x values in order", [f.targets[0].x_mm for f in frames] == [50, 150, 250])


def test_resync_after_garbage():
    parser = FrameParser()
    noise = bytes.fromhex("00ffaaff0311") + bytes([0x03] * 6)
    stream = noise + build_report_frame([(77, 900, 0)]) + noise
    frames = parser.feed(stream)
    check("resyncs past garbage and a fake header", len(frames) == 1)
    check("post-garbage x_mm correct", frames[0].targets[0].x_mm == 77)
    check("garbage was counted", parser.bytes_discarded > 0)


def test_rejects_bad_tail():
    frame = bytearray(build_report_frame([(1, 1, 0)]))
    frame[-1] = 0x00  # tail must be 55 CC
    check("bad tail rejected", FrameParser().feed(bytes(frame)) == [])


def test_short_body_rejected():
    check("short body rejected", parse_frame_body(b"\x00" * 10) is None)


def test_magnitude_clamped_at_encode():
    # Values beyond the 15-bit magnitude range must not corrupt the sign bit
    # when building a test frame.
    frame = build_report_frame([(40000, -40000, 0)])
    decoded = FrameParser().feed(frame)[0]
    check("x clamped to max magnitude", decoded.targets[0].x_mm == ld2450.MAGNITUDE_MASK)
    check("y clamped to -max magnitude", decoded.targets[0].y_mm == -ld2450.MAGNITUDE_MASK)


def test_verify_sign_convention_helper():
    check("left is negative x", ld2450.verify_sign_convention(-500, "left") is True)
    check("right is positive x", ld2450.verify_sign_convention(500, "right") is True)
    check("mismatched direction is False", ld2450.verify_sign_convention(500, "left") is False)
    try:
        ld2450.verify_sign_convention(0, "sideways")
        check("bad direction raises", False)
    except ValueError:
        check("bad direction raises", True)


def _frame(targets, t):
    return parse_frame_body(build_report_frame(targets)[4:-2], timestamp=t)


def test_smoother_first_reading_is_exact():
    sm = ld2450.TargetSmoother()
    sm.update(_frame([(300, 2000, 0)], 0.0))
    (t,) = sm.targets(0.0)
    check("first reading not smoothed", (t.x_mm, t.y_mm) == (300, 2000) and t.slot == 1 and not t.held)


def test_smoother_damps_jitter():
    sm = ld2450.TargetSmoother(alpha=0.5)
    sm.update(_frame([(0, 2000, 0)], 0.0))
    sm.update(_frame([(200, 2000, 0)], 0.1))
    (t,) = sm.targets(0.1)
    check("jitter halved at alpha 0.5", t.x_mm == 100)


def test_smoother_resets_on_big_jump():
    sm = ld2450.TargetSmoother(alpha=0.5)
    sm.update(_frame([(0, 1000, 0)], 0.0))
    sm.update(_frame([(0, 3000, 0)], 0.1))
    (t,) = sm.targets(0.1)
    check("big jump snaps instead of sliding", t.y_mm == 3000)


def test_smoother_holds_then_drops():
    sm = ld2450.TargetSmoother(hold_seconds=1.0)
    sm.update(_frame([(0, 2000, 25)], 0.0))
    sm.update(_frame([], 0.5))
    (t,) = sm.targets(0.5)
    check("lost target held at last position", t.held and t.y_mm == 2000)
    check("held target reported as not moving", t.speed_cms == 0)
    check("held target gone after hold time", sm.targets(1.2) == [])


def test_smoother_reset():
    sm = ld2450.TargetSmoother()
    sm.update(_frame([(0, 2000, 0)], 0.0))
    sm.reset()
    check("reset clears targets", sm.targets(0.0) == [])


def _walk(sm, far_of, frames=15, step_mm=40):
    """One person walking straight away from the sensor, plus a second
    target whose distance far_of(i, near_y) gives; returns targets()."""
    for i in range(frames):
        near_y = 1000 + i * step_mm
        sm.update(_frame([(0, near_y, 20), (0, far_of(i, near_y), 0)], i * 0.09))
    return sm.targets((frames - 1) * 0.09)


def test_echo_flagged_when_moving_in_step():
    near, far = _walk(ld2450.TargetSmoother(), lambda i, y: y + 1500)
    check("farther in-step target flagged as echo", far.echo)
    check("nearer target never flagged", not near.echo)


def test_echo_not_flagged_for_independent_target():
    wobble = [0, 30, -20, 10, -30, 25, 0, -10, 20, -25, 15, 5, -15, 30, -5]
    near, far = _walk(ld2450.TargetSmoother(), lambda i, y: 3000 + wobble[i])
    check("unrelated still target not an echo", not near.echo and not far.echo)


def test_echo_not_flagged_when_nearer_is_still():
    sm = ld2450.TargetSmoother()
    for i in range(15):
        sm.update(_frame([(0, 1000 + (i % 2) * 10, 0), (0, 2500 + (i % 2) * 10, 0)], i * 0.09))
    check("no echo call without real motion", not any(t.echo for t in sm.targets(14 * 0.09)))


def test_echo_needs_enough_history():
    near, far = _walk(ld2450.TargetSmoother(), lambda i, y: y + 1500, frames=5)
    check("too little history is never an echo", not far.echo)


def test_frame_recorder_writes_csv():
    import tempfile
    from pathlib import Path

    from fieldlog import FrameRecorder

    with tempfile.TemporaryDirectory() as tmp:
        rec = FrameRecorder(log_dir=Path(tmp))
        rec.on_frame(_frame([(-120, 2300, 15)], 12.5), sensor="S1")
        rec.close()
        rec.on_frame(_frame([(1, 1, 1)], 13.0), sensor="S1")  # after close: ignored
        lines = rec.path.read_text(encoding="utf-8").splitlines()
    check("recorder header", lines[0] == "t,sensor,x1,y1,s1,x2,y2,s2,x3,y3,s3")
    check("recorder row", lines[1] == "12.500,S1,-120,2300,15,0,0,0,0,0,0")
    check("recorder ignores frames after close", len(lines) == 2 and rec.frames == 1)


if __name__ == "__main__":
    for name, fn in sorted(list(globals().items())):
        if name.startswith("test_"):
            fn()
    for label in PASSED:
        print("  ok  " + label)
    print("\n%d checks passed" % len(PASSED))
