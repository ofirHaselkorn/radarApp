"""LD2410C config/command protocol tests. Run with: python test_ld2410_config.py

Covers frame building/parsing and the AckParser's stream handling (all pure
protocol logic, no hardware needed), plus ConfigSession's request/response
and error-handling logic against a lightweight fake transport that plays
back the responses this module expects. That fake transport validates
ConfigSession's own code -- the timeout, the success/failure check, the
command-word matching -- not that the real sensor actually behaves this
way; see the CONFIDENCE NOTE at the top of ld2410_config.py for what still
needs a real-hardware check.
"""

from collections import deque

import ld2410_config as cfg

PASSED = []


def check(label, condition):
    assert condition, "FAILED: " + label
    PASSED.append(label)


def build_ack(command_word, success=True, value=b""):
    """Build a raw ACK frame the way the sensor would send one."""
    ack_word = (command_word | cfg.ACK_BIT).to_bytes(2, "little")
    status = (0 if success else 1).to_bytes(2, "little")
    data = ack_word + status + value
    return cfg.FRAME_HEADER + len(data).to_bytes(2, "little") + data + cfg.FRAME_FOOTER


def test_command_frame_envelope():
    frame = cfg.build_command_frame(cfg.CMD_ENABLE_CONFIG, cfg.PROTOCOL_VERSION)
    check("starts with FD FC FB FA", frame[:4] == cfg.FRAME_HEADER)
    check("ends with 04 03 02 01", frame[-4:] == cfg.FRAME_FOOTER)
    check("command word little endian", frame[6:8] == cfg.CMD_ENABLE_CONFIG.to_bytes(2, "little"))
    check("value present", frame[8:10] == cfg.PROTOCOL_VERSION)


def test_ack_round_trip():
    raw = build_ack(cfg.CMD_READ_FIRMWARE_VERSION, success=True, value=b"\x01\x02\x03\x04")
    frames = cfg.AckParser().feed(raw)
    check("one ack decoded", len(frames) == 1)
    ack = frames[0]
    check("command word recovered (ack bit stripped)", ack.command_word == cfg.CMD_READ_FIRMWARE_VERSION)
    check("success flag true", ack.success is True)
    check("value bytes preserved", ack.value == b"\x01\x02\x03\x04")


def test_ack_failure_status():
    raw = build_ack(cfg.CMD_SET_GATE_SENSITIVITY, success=False)
    ack = cfg.AckParser().feed(raw)[0]
    check("nonzero status decodes as failure", ack.success is False)


def test_ack_resync_after_garbage():
    parser = cfg.AckParser()
    noise = bytes.fromhex("00fffdfc1122") + bytes([0xAA] * 6)
    stream = noise + build_ack(cfg.CMD_ENABLE_CONFIG) + noise
    frames = parser.feed(stream)
    check("resyncs past garbage with a fake header", len(frames) == 1)
    check("garbage counted as discarded", parser.bytes_discarded > 0)


def test_ack_byte_split_stream():
    parser = cfg.AckParser()
    raw = build_ack(cfg.CMD_RESTART)
    decoded = []
    for i in range(len(raw)):
        decoded.extend(parser.feed(raw[i:i + 1]))
    check("split stream yields one ack", len(decoded) == 1)


def test_not_an_ack_rejected():
    # A frame with the ACK bit clear in its "ack word" position is not a
    # valid ACK (e.g. a command frame looped back on a loopback test rig).
    check("ack bit required", cfg.parse_ack_body(b"\x60\x00\x00\x00") is None)
    check("short body rejected", cfg.parse_ack_body(b"\x00") is None)


def test_set_max_gate_duration_frame():
    frame = cfg.build_set_max_gate_duration(max_moving_gate=8, max_static_gate=6, duration_s=5)
    check("frame well-formed", frame[:4] == cfg.FRAME_HEADER and frame[-4:] == cfg.FRAME_FOOTER)
    check("rejects out-of-range moving gate",
          _raises(lambda: cfg.build_set_max_gate_duration(9, 6, 5)))
    check("rejects out-of-range static gate",
          _raises(lambda: cfg.build_set_max_gate_duration(8, 9, 5)))


def test_set_gate_sensitivity_frame():
    frame = cfg.build_set_gate_sensitivity(gate=3, motion_pct=50, static_pct=40)
    check("gate sensitivity frame well-formed", frame[:4] == cfg.FRAME_HEADER)
    check("GATE_ALL accepted", cfg.build_set_gate_sensitivity(cfg.GATE_ALL, 50, 50))
    check("rejects bad gate index", _raises(lambda: cfg.build_set_gate_sensitivity(99, 50, 50)))
    check("rejects bad percentage", _raises(lambda: cfg.build_set_gate_sensitivity(0, 150, 50)))


def test_read_parameters_decode():
    value = bytes([0xAA, 8, 6]) + bytes(range(9)) + bytes(range(50, 59)) + (5).to_bytes(2, "little") + bytes([0x55])
    params = cfg.parse_read_parameters(value)
    check("max_moving_gate decoded", params.max_moving_gate == 8)
    check("max_static_gate decoded", params.max_static_gate == 6)
    check("motion sensitivity array decoded", params.motion_sensitivity == tuple(range(9)))
    check("static sensitivity array decoded", params.static_sensitivity == tuple(range(50, 59)))
    check("duration decoded", params.duration_s == 5)


def test_read_parameters_bad_markers_rejected():
    bad = bytes([0x00]) + bytes(22) + bytes([0x55])
    check("missing AA marker raises", _raises(lambda: cfg.parse_read_parameters(bad)))


def _raises(fn):
    try:
        fn()
        return False
    except (ValueError, cfg.ConfigError):
        return True


class FakeTransport:
    """Minimal fake link: queues canned ACK responses, records writes."""

    def __init__(self):
        self.written = []
        self._pending = deque()

    def queue_ack(self, command_word, success=True, value=b""):
        self._pending.append(build_ack(command_word, success, value))

    def write(self, data: bytes) -> None:
        self.written.append(bytes(data))

    def read(self, _max_bytes: int = 256) -> bytes:
        if self._pending:
            return self._pending.popleft()
        return b""

    def close(self) -> None:
        pass


def test_config_session_happy_path():
    transport = FakeTransport()
    transport.queue_ack(cfg.CMD_ENABLE_CONFIG, value=b"\x01\x00\x40\x00")
    transport.queue_ack(cfg.CMD_SET_MAX_GATE_DURATION)
    transport.queue_ack(cfg.CMD_END_CONFIG)

    with cfg.ConfigSession(transport, timeout=1.0) as session:
        session.set_max_gate_duration(8, 8, 5)

    check("enter config frame sent first", transport.written[0][6:8] == cfg.CMD_ENABLE_CONFIG.to_bytes(2, "little"))
    check("set-gate-duration frame sent second",
          transport.written[1][6:8] == cfg.CMD_SET_MAX_GATE_DURATION.to_bytes(2, "little"))
    check("end config sent on context exit",
          transport.written[2][6:8] == cfg.CMD_END_CONFIG.to_bytes(2, "little"))


def test_config_session_failure_raises():
    transport = FakeTransport()
    transport.queue_ack(cfg.CMD_ENABLE_CONFIG)
    transport.queue_ack(cfg.CMD_SET_GATE_SENSITIVITY, success=False)
    transport.queue_ack(cfg.CMD_END_CONFIG)

    raised = False
    with cfg.ConfigSession(transport, timeout=1.0) as session:
        try:
            session.set_gate_sensitivity(0, 50, 50)
        except cfg.ConfigError:
            raised = True
    check("a failure status raises ConfigError", raised)


def test_config_session_timeout_raises():
    transport = FakeTransport()
    transport.queue_ack(cfg.CMD_ENABLE_CONFIG)
    # No ACK queued for read_parameters -> must time out, not hang forever.
    transport.queue_ack(cfg.CMD_END_CONFIG)

    raised = False
    with cfg.ConfigSession(transport, timeout=0.3) as session:
        try:
            session.read_parameters()
        except cfg.ConfigError as exc:
            raised = "within" in str(exc)
    check("a missing ack times out with a clear message", raised)


def test_config_session_read_parameters_end_to_end():
    transport = FakeTransport()
    transport.queue_ack(cfg.CMD_ENABLE_CONFIG)
    value = bytes([0xAA, 8, 8]) + bytes([50] * 9) + bytes([40] * 9) + (10).to_bytes(2, "little") + bytes([0x55])
    transport.queue_ack(cfg.CMD_READ_PARAMETERS, value=value)
    transport.queue_ack(cfg.CMD_END_CONFIG)

    with cfg.ConfigSession(transport, timeout=1.0) as session:
        params = session.read_parameters()
    check("end-to-end max_moving_gate", params.max_moving_gate == 8)
    check("end-to-end duration", params.duration_s == 10)


if __name__ == "__main__":
    for name, fn in sorted(list(globals().items())):
        if name.startswith("test_"):
            fn()
    for label in PASSED:
        print("  ok  " + label)
    print("\n%d checks passed" % len(PASSED))
