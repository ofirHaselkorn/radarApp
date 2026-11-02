"""Protocol tests. Run with: python test_parser.py (no dependencies needed)."""

import ld2410
from ld2410 import FrameParser, build_basic_frame, parse_payload

# A hand-written basic frame: stationary target at 0x0100 = 256 cm, energy 0x3C.
REFERENCE = bytes.fromhex("f4f3f2f1" "0d00" "02" "aa" "02"
                          "0000" "00" "0001" "3c" "0001" "55" "00" "f8f7f6f5")

PASSED = []


def check(label, condition):
    assert condition, "FAILED: " + label
    PASSED.append(label)


def test_reference_frame():
    check("reference frame is 23 bytes", len(REFERENCE) == 23)
    frames = FrameParser().feed(REFERENCE)
    check("reference frame decodes", len(frames) == 1)
    f = frames[0]
    check("state is stationary", f.state == ld2410.STATE_STATIONARY)
    check("little endian 00 01 -> 256 cm", f.stationary_distance_cm == 256)
    check("stationary energy 0x3C -> 60", f.stationary_energy == 60)
    check("detection distance 256 cm", f.detection_distance_cm == 256)
    check("reported distance is the stationary one", f.distance_cm == 256)
    check("stationary target flagged", f.has_stationary and not f.has_moving)


def test_all_states():
    cases = {
        ld2410.STATE_NO_TARGET: (None, False),
        ld2410.STATE_MOVING: (137, True),
        ld2410.STATE_STATIONARY: (241, True),
        ld2410.STATE_BOTH: (137, True),
    }
    for state, (expect_distance, expect_target) in cases.items():
        frame = FrameParser().feed(
            build_basic_frame(
                state,
                moving_distance_cm=137,
                moving_energy=55,
                stationary_distance_cm=241,
                stationary_energy=70,
                detection_distance_cm=137,
            )
        )[0]
        check("state 0x%02X has_target" % state, frame.has_target == expect_target)
        check("state 0x%02X distance" % state, frame.distance_cm == expect_distance)
    check("moving wins over stationary when both",
          FrameParser().feed(build_basic_frame(ld2410.STATE_BOTH, 137, 55, 241, 70))[0]
          .distance_cm == 137)


def test_byte_split_stream():
    """One frame delivered one byte at a time must still decode exactly once."""
    parser = FrameParser()
    frame = build_basic_frame(ld2410.STATE_MOVING, 480, 90, detection_distance_cm=480)
    decoded = []
    for i in range(len(frame)):
        decoded.extend(parser.feed(frame[i:i + 1]))
    check("split stream yields one frame", len(decoded) == 1)
    check("split stream distance 480 cm", decoded[0].moving_distance_cm == 480)


def test_resync_after_garbage():
    parser = FrameParser()
    noise = bytes.fromhex("00ff11f4f3f2") + bytes([0xAA] * 9)
    stream = noise + build_basic_frame(ld2410.STATE_MOVING, 75, 100) + noise
    frames = parser.feed(stream)
    check("resyncs past garbage and a fake header", len(frames) == 1)
    check("post-garbage distance 75 cm", frames[0].moving_distance_cm == 75)
    check("garbage was counted", parser.bytes_discarded > 0)


def test_back_to_back_frames():
    parser = FrameParser()
    stream = b"".join(
        build_basic_frame(ld2410.STATE_MOVING, d, 50, detection_distance_cm=d)
        for d in (100, 200, 300, 65535)
    )
    frames = parser.feed(stream)
    check("four frames in one read", len(frames) == 4)
    check("distances in order",
          [f.moving_distance_cm for f in frames] == [100, 200, 300, 65535])
    check("max 16 bit distance survives", frames[-1].moving_distance_cm == 0xFFFF)


def test_rejects_malformed():
    bad_head = bytearray(REFERENCE)
    bad_head[7] = 0xAB  # head must be AA
    check("bad payload head rejected", FrameParser().feed(bytes(bad_head)) == [])

    bad_tail = bytearray(REFERENCE)
    bad_tail[17] = 0x56  # tail must be 55
    check("bad payload tail rejected", FrameParser().feed(bytes(bad_tail)) == [])

    bad_footer = bytearray(REFERENCE)
    bad_footer[22] = 0x00
    check("bad frame footer rejected", FrameParser().feed(bytes(bad_footer)) == [])

    wrong_type = bytearray(REFERENCE)
    wrong_type[6] = 0x07  # not a data frame
    check("unknown data type rejected", FrameParser().feed(bytes(wrong_type)) == [])

    check("short payload rejected", parse_payload(b"\x02\xaa\x01") is None)


def test_engineering_frame_accepted():
    """Engineering mode (type 01) carries the same first 11 payload bytes."""
    payload = bytearray([0x01, 0xAA, ld2410.STATE_MOVING,
                         0x2C, 0x01, 0x5A,  # 300 cm, energy 90
                         0x00, 0x00, 0x00,
                         0x2C, 0x01])       # detection 300 cm
    payload += bytes(14)                    # per-gate data
    payload += bytes([0x55, 0x00])
    raw = (ld2410.FRAME_HEADER + len(payload).to_bytes(2, "little")
           + bytes(payload) + ld2410.FRAME_FOOTER)
    frames = FrameParser().feed(raw)
    check("engineering frame decodes", len(frames) == 1)
    check("engineering distance 300 cm", frames[0].moving_distance_cm == 300)


def test_buffer_is_bounded():
    parser = FrameParser(max_buffer=512)
    for _ in range(50):
        parser.feed(b"\x00" * 256)
    check("buffer stays bounded under pure noise", len(parser._buf) <= 512)
    frames = parser.feed(build_basic_frame(ld2410.STATE_MOVING, 123, 40))
    check("still parses after noise flood", len(frames) == 1
          and frames[0].moving_distance_cm == 123)


if __name__ == "__main__":
    for name, fn in sorted(list(globals().items())):
        if name.startswith("test_"):
            fn()
    for label in PASSED:
        print("  ok  " + label)
    print("\n%d checks passed" % len(PASSED))
