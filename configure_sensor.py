"""Standalone CLI for the LD2410C configuration protocol (ld2410_config.py).

Needs the port to itself -- the sensor doesn't stream ordinary data frames
during a config session, so this cannot run while main.py (or anything
else) has the same port open. Close the app first.

    python configure_sensor.py --port COM5 read
    python configure_sensor.py --port COM5 set-gates --moving 8 --static 6 --duration 5
    python configure_sensor.py --port COM5 set-sensitivity --gate all --motion 50 --static 40
    python configure_sensor.py --port COM5 firmware
    python configure_sensor.py --port COM5 restart
    python configure_sensor.py --port COM5 factory-reset     # asks to confirm

With no --port, the first recognised USB-TTL adapter is used (same
auto-detect as main.py/ld2410.py).
"""

from __future__ import annotations

import argparse
import sys

from ld2410_config import ConfigError, ConfigSession, GATE_ALL, MAX_GATE_INDEX
from serial_backend import DEFAULT_BAUD, SerialLinkError, open_link


def _gate_arg(value: str) -> int:
    if value.lower() == "all":
        return GATE_ALL
    gate = int(value)
    if not 0 <= gate <= MAX_GATE_INDEX:
        raise argparse.ArgumentTypeError("gate must be 0-%d or 'all'" % MAX_GATE_INDEX)
    return gate


def _cmd_read(session: ConfigSession, _args) -> None:
    params = session.read_parameters()
    print("max moving gate:    %d" % params.max_moving_gate)
    print("max static gate:    %d" % params.max_static_gate)
    print("no-one duration:    %d s" % params.duration_s)
    print("motion sensitivity: " + " ".join("%3d" % v for v in params.motion_sensitivity))
    print("static sensitivity: " + " ".join("%3d" % v for v in params.static_sensitivity))
    print("                    " + " ".join("g%d " % i for i in range(len(params.motion_sensitivity))))


def _cmd_set_gates(session: ConfigSession, args) -> None:
    session.set_max_gate_duration(args.moving, args.static, args.duration)
    print("ok: max moving gate %d, max static gate %d, duration %ds" % (args.moving, args.static, args.duration))


def _cmd_set_sensitivity(session: ConfigSession, args) -> None:
    session.set_gate_sensitivity(args.gate, args.motion, args.static)
    gate_label = "all gates" if args.gate == GATE_ALL else "gate %d" % args.gate
    print("ok: %s motion=%d static=%d" % (gate_label, args.motion, args.static))


def _cmd_firmware(session: ConfigSession, _args) -> None:
    raw = session.read_firmware_version()
    print("raw firmware reply: " + raw.hex(" "))
    print(
        "(this module is less certain of the exact field layout here -- "
        "see read_firmware_version()'s docstring in ld2410_config.py. "
        "Compare this against what the sensor's own configuration tool shows.)"
    )


def _cmd_restart(session: ConfigSession, _args) -> None:
    session.restart()
    print("ok: restart command sent")


def _cmd_factory_reset(session: ConfigSession, args) -> None:
    if not args.yes:
        answer = input("This erases all sensor settings. Type 'yes' to confirm: ")
        if answer.strip().lower() != "yes":
            print("cancelled")
            return
    session.factory_reset()
    print("ok: factory reset command sent")


def _build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--timeout", type=float, default=2.0, help="seconds to wait for each ACK")

    sub = ap.add_subparsers(dest="command", required=True)

    sub.add_parser("read", help="read current gate/sensitivity/duration settings").set_defaults(func=_cmd_read)

    p = sub.add_parser("set-gates", help="set max moving/static gate and no-one duration")
    p.add_argument("--moving", type=int, required=True, help="0-%d" % MAX_GATE_INDEX)
    p.add_argument("--static", type=int, required=True, help="0-%d" % MAX_GATE_INDEX)
    p.add_argument("--duration", type=int, required=True, help="seconds before 'no one' is declared")
    p.set_defaults(func=_cmd_set_gates)

    p = sub.add_parser("set-sensitivity", help="set motion/static sensitivity for one gate or all")
    p.add_argument("--gate", type=_gate_arg, required=True, help="0-%d or 'all'" % MAX_GATE_INDEX)
    p.add_argument("--motion", type=int, required=True, help="0-100")
    p.add_argument("--static", type=int, required=True, help="0-100")
    p.set_defaults(func=_cmd_set_sensitivity)

    sub.add_parser("firmware", help="read firmware version (raw bytes, see caveat)").set_defaults(func=_cmd_firmware)
    sub.add_parser("restart", help="restart the sensor module").set_defaults(func=_cmd_restart)

    p = sub.add_parser("factory-reset", help="erase all settings (asks to confirm)")
    p.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    p.set_defaults(func=_cmd_factory_reset)

    return ap


def main() -> None:
    args = _build_parser().parse_args()

    try:
        link = open_link(port=args.port, baud=args.baud)
    except SerialLinkError as exc:
        print("error: %s" % exc, file=sys.stderr)
        sys.exit(1)

    print("opened %s" % link.description)
    try:
        with ConfigSession(link, timeout=args.timeout) as session:
            args.func(session, args)
    except ConfigError as exc:
        print("error: %s" % exc, file=sys.stderr)
        sys.exit(1)
    finally:
        link.close()


if __name__ == "__main__":
    main()
