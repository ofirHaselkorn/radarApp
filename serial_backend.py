"""Serial links for the LD2410C/LD2450, one API across Android, desktop and mock.

Every link exposes the slice of the pyserial API callers need:

    link.read(max_bytes) -> bytes   (returns b"" on timeout, never blocks long)
    link.write(data: bytes) -> None (only needed for ld2410_config.py's commands;
                                      the normal read-only data stream never calls it)
    link.close()
    link.description                (human readable, shown in the UI)

Backends:
    * Android  -> usb4a + usbserial4a (CH340 / CP210x / FTDI / CDC-ACM)
    * Desktop  -> pyserial, with CH340 auto-detection
    * Mock     -> synthetic frames, so the UI can be developed with no hardware
"""

from __future__ import annotations

import math
import os
import time
from typing import Callable, Optional

from ld2410 import (
    STATE_BOTH,
    STATE_MOVING,
    STATE_NO_TARGET,
    STATE_STATIONARY,
    build_basic_frame,
)
from ld2450 import build_report_frame
from sensor_detect import SENSOR_LD2450

# Factory defaults of the LD2410C UART.
DEFAULT_BAUD = 256000
DEFAULT_BYTESIZE = 8
DEFAULT_PARITY = "N"
DEFAULT_STOPBITS = 1
# Short read timeout: it bounds how fast the reader thread notices a stop
# request, and it is what makes read() return b"" instead of blocking forever.
READ_TIMEOUT = 0.2

# USB-serial bridges we know how to talk to, most likely first. 0x1A86/0x7523
# is the CH340 in the Type-C adapters sold with these radars.
KNOWN_USB_IDS = (
    (0x1A86, None),  # WCH CH340 / CH341 / CH9102
    (0x10C4, None),  # Silicon Labs CP210x
    (0x0403, None),  # FTDI
    (0x067B, None),  # Prolific PL2303
)


def is_android() -> bool:
    """True when running inside a python-for-android / Kivy APK."""
    return "ANDROID_ARGUMENT" in os.environ or "ANDROID_BOOTLOGO" in os.environ


class SerialLinkError(RuntimeError):
    """Raised when a port cannot be opened; the reader retries on this."""


class PortLink:
    """Wraps a pyserial-compatible port (desktop pyserial or usbserial4a)."""

    def __init__(self, port, description: str, capture_path: Optional[str] = None):
        self._port = port
        self.description = description
        # Optional raw dump of the first RAW_CAPTURE_BYTES of each connection,
        # so what the phone actually receives can be pulled off with adb.
        self._capture = None
        if capture_path:
            try:
                self._capture = open(capture_path, "wb")
            except OSError:
                self._capture = None
        self._captured = 0

    def read(self, max_bytes: int = 256) -> bytes:
        # Drain whatever is buffered in one go; otherwise fall back to a single
        # timeout-bounded byte read so the loop keeps ticking when idle.
        try:
            waiting = self._port.in_waiting
        except Exception:
            waiting = 0
        count = min(waiting, max_bytes) if waiting else 1
        data = self._port.read(count)
        data = bytes(data) if data else b""
        if data and self._capture is not None:
            self._capture.write(data[: RAW_CAPTURE_BYTES - self._captured])
            self._captured += len(data)
            if self._captured >= RAW_CAPTURE_BYTES:
                self._capture.close()
                self._capture = None
        return data

    def write(self, data: bytes) -> None:
        """Only needed for sending commands (ld2410_config.py); the normal
        read-only data-streaming path (LD2410Reader) never calls this."""
        self._port.write(data)

    def close(self) -> None:
        if self._capture is not None:
            self._capture.close()
            self._capture = None
        try:
            self._port.close()
        except Exception:
            pass


class MockLink:
    """Synthetic sensor: a target that walks in and out of range.

    Emits real wire-format frames at ~15 Hz, so the parser is exercised end to
    end and the UI behaves exactly as it will with hardware attached.

    phase_offset staggers the cycle so a second (third, ...) mock sensor in a
    multi-sensor setup doesn't mirror the first one exactly -- in real life
    two sensors watching different directions see different things.
    """

    FRAME_INTERVAL = 1.0 / 15.0

    def __init__(self, description: str = "MOCK SENSOR (no hardware)", phase_offset: float = 0.0):
        self.description = description
        self._phase_offset = phase_offset
        self._t0 = time.time()
        self._next = self._t0

    def read(self, max_bytes: int = 256) -> bytes:
        now = time.time()
        if now < self._next:
            time.sleep(min(self._next - now, READ_TIMEOUT))
            return b""
        self._next = max(now, self._next + self.FRAME_INTERVAL)

        # 20 s cycle: approach, linger, leave, empty room.
        phase = (now - self._t0 + self._phase_offset) % 20.0
        if phase < 7.0:  # walking in from 500 cm to 80 cm
            distance = int(500 - (500 - 80) * (phase / 7.0))
            energy = int(35 + 60 * (phase / 7.0))
            return build_basic_frame(
                STATE_MOVING,
                moving_distance_cm=distance,
                moving_energy=min(energy, 100),
                detection_distance_cm=distance,
            )
        if phase < 12.0:  # standing still, breathing
            distance = 80 + int(6 * math.sin(now * 2.0))
            return build_basic_frame(
                STATE_BOTH if int(phase) % 2 else STATE_STATIONARY,
                moving_distance_cm=distance,
                moving_energy=22,
                stationary_distance_cm=distance,
                stationary_energy=68,
                detection_distance_cm=distance,
            )
        if phase < 17.0:  # walking back out
            distance = int(80 + (520 - 80) * ((phase - 12.0) / 5.0))
            return build_basic_frame(
                STATE_MOVING,
                moving_distance_cm=distance,
                moving_energy=max(100 - int(70 * (phase - 12.0) / 5.0), 10),
                detection_distance_cm=distance,
            )
        return build_basic_frame(STATE_NO_TARGET)  # clear

    def write(self, data: bytes) -> None:
        # The mock has no config state machine to drive -- ld2410_config.py
        # against a MockLink would just time out waiting for an ACK that
        # never comes, which is honest (there's nothing real to configure).
        pass

    def close(self) -> None:
        pass


class MockLD2450Link:
    """Synthetic LD2450: two people moving around in front of the sensor.

    Emits real 30 byte LD2450 report frames at ~10 Hz (the real sensor's
    rate), so the auto-detecting parser, the XY plotting and the target
    readout all run exactly as they will on hardware. Target 1 paces left
    and right across the field of view; target 2 walks in, stands still for
    a while, and walks back out, leaving one slot empty part of the time.
    """

    FRAME_INTERVAL = 1.0 / 10.0

    def __init__(self, description: str = "MOCK LD2450 (no hardware)", phase_offset: float = 0.0):
        self.description = description
        self._phase_offset = phase_offset
        self._t0 = time.time()
        self._next = self._t0

    def read(self, max_bytes: int = 256) -> bytes:
        now = time.time()
        if now < self._next:
            time.sleep(min(self._next - now, READ_TIMEOUT))
            return b""
        self._next = max(now, self._next + self.FRAME_INTERVAL)
        t = now - self._t0 + self._phase_offset

        targets = []
        # Target 1: a slow arc 2.5 m out, sweeping about +/-45 deg.
        angle = math.radians(45.0 * math.sin(t * 0.35))
        speed = int(2500 * 0.35 * math.radians(45.0) * math.cos(t * 0.35) / 10.0)
        targets.append((int(2500 * math.sin(angle)), int(2500 * math.cos(angle)), speed))

        # Target 2: 16 s cycle -- in from 5 m, stand at 1.2 m, back out, gone.
        phase = t % 16.0
        if phase < 5.0:
            y = int(5000 - 3800 * (phase / 5.0))
            targets.append((-900, y, -76))
        elif phase < 9.0:
            targets.append((-900, 1200, 0))
        elif phase < 13.0:
            y = int(1200 + 3800 * ((phase - 9.0) / 4.0))
            targets.append((-900, y, 95))
        return build_report_frame(targets)

    def write(self, data: bytes) -> None:
        pass

    def close(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Android: usb4a + usbserial4a
# ---------------------------------------------------------------------------

PERMISSION_TIMEOUT = 20.0

# usbserial4a's CH34x driver only knows a fixed table of standard rates and
# silently rounds anything else *up* to the next entry -- 256000 (the
# radar's rate) lands on 460800, so the link "connects" but every byte is
# garbage and no frame ever parses. Found on real hardware: phone showed a
# live connection and zero detections. These are the rates its table has
# exactly; anything else goes through ch34x_divisor_register() instead.
_CH34X_TABLE_RATES = frozenset(
    (300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)
)
_CH34X_CLKRATE = 48_000_000

# Register pairs (0x1312, 0x0F2C) measured on real hardware, which win over
# ch34x_divisor_register(). 256000: the computed value (12 MHz / 47, nominally
# 0.27% off) delivered ~2% clean frames on a Galaxy S25 + CH340G + LD2410C --
# every error a 0 bit read as 1 just before a 1 bit, i.e. sampling late. A
# sweep with the CH340 run from its 6 MHz base found clean data from ~261k to
# ~273k and failures at 250k/258k/286k, so this pair links at whatever the
# sensor actually transmits, ~4% above nominal. The value used is the middle
# of that window: 6 MHz * 256 / (0x10000 - 0xE980) = ~266.7k, 100% clean
# frames (178/178). 0x0F2C acts as the divisor's fractional low byte here.
_CH34X_MEASURED_REGS = {256000: (0xE983, 0x80)}


def ch34x_divisor_register(baud: int) -> int:
    """Value for CH34x register pair 0x1312 (divisor << 8 | prescaler) that
    gives the closest rate to baud. A port of the Linux kernel's
    ch341_get_divisor() (drivers/usb/serial/ch341.c), plus the bit 7 that
    usbserial4a's own table always sets (disables the chip's 32-byte
    buffering). Reproduces usbserial4a's table values for the rates it has.

    Only a fallback for rates with no measured entry in _CH34X_MEASURED_REGS:
    for 256000 it gives 0xD187, which looks right on paper but corrupted
    most bytes on real hardware (see that table's comment).
    """
    def clk_div(ps: int, fact: int) -> int:
        return 1 << (12 - 3 * ps - fact)

    baud = max(46, min(2_000_000, baud))
    fact = 1
    for ps in range(3, -1, -1):
        if baud > _CH34X_CLKRATE // (clk_div(ps, 1) * 512):
            break
    else:
        raise ValueError("baud %d too low for CH34x" % baud)

    div_clk = clk_div(ps, fact)
    div = _CH34X_CLKRATE // (div_clk * baud)
    if div < 9 or div > 255:
        div //= 2
        div_clk *= 2
        fact = 0
    if div < 2:
        raise ValueError("baud %d too high for CH34x" % baud)
    # Round to the nearer divisor (scaled by 16 to avoid rounding errors).
    if (16 * _CH34X_CLKRATE // (div_clk * div) - 16 * baud
            >= 16 * baud - 16 * _CH34X_CLKRATE // (div_clk * (div + 1))):
        div += 1
    # Prefer the lower base clock when the divisor is even.
    if fact == 1 and div % 2 == 0:
        div //= 2
        fact = 0
    return ((0x100 - div) << 8) | (fact << 2) | ps | 0x80


RAW_CAPTURE_BYTES = 8192
RAW_CAPTURE_FILE = "raw_capture.bin"
# Debug hook, in the app's private files dir: {"reg1312": int, "reg0f2c": int}
# forces those two CH34x baud registers for every rate, so divisor variants
# can be tried over adb (run-as ... write the file, restart the app) without
# rebuilding the APK. Delete the file to go back to the computed values.
CH34X_OVERRIDE_FILE = "ch34x_baud.json"


def _android_files_dir() -> Optional[str]:
    try:
        from android.storage import app_storage_path  # type: ignore

        return app_storage_path()
    except Exception:
        return None


def _read_ch34x_override() -> Optional[tuple]:
    files_dir = _android_files_dir()
    if not files_dir:
        return None
    try:
        import json

        with open(os.path.join(files_dir, CH34X_OVERRIDE_FILE)) as fh:
            data = json.load(fh)
        return int(data["reg1312"]), int(data["reg0f2c"])
    except Exception:
        return None


def _patch_ch34x_baudrate() -> None:
    """Teach usbserial4a's CH34x driver non-table baud rates (once)."""
    from usbserial4a.ch34xserial4a import Ch34xSerial, SerialException

    if getattr(Ch34xSerial, "_ld2410_patched", False):
        return
    original = Ch34xSerial._set_baudrate

    def _set_baudrate(self, baudrate):
        override = _read_ch34x_override()
        if override is None and (baudrate in _CH34X_TABLE_RATES or baudrate <= 19200):
            return original(self, baudrate)
        if override is not None:
            reg1312, reg0f2c = override
        elif baudrate in _CH34X_MEASURED_REGS:
            reg1312, reg0f2c = _CH34X_MEASURED_REGS[baudrate]
        else:
            reg1312, reg0f2c = ch34x_divisor_register(baudrate), self.CH34X_19200_0f2c_rest
        # Same register sequence as the original, with a computed divisor.
        if self._ctrl_transfer_out(self.CH34X_REQ_WRITE_REG, 0x1312, reg1312) < 0:
            raise SerialException("Fail to set baudrate index1312!")
        if self._ctrl_transfer_out(self.CH34X_REQ_WRITE_REG, 0x0F2C, reg0f2c) < 0:
            raise SerialException("Fail to set baudrate index0f2c!")
        self._check_state("Set baudrate", self.CH34X_REQ_READ_REG, 0x0706, [0x9F, 0xEE])
        if self._ctrl_transfer_out(self.CH34X_REQ_WRITE_REG, 0x2727, 0) < 0:
            raise SerialException("Fail to set baudrate!")

    Ch34xSerial._set_baudrate = _set_baudrate
    Ch34xSerial._ld2410_patched = True


def _rank_android_device(device) -> int:
    """Lower is better: prefer a known USB-serial bridge over anything else."""
    try:
        vid = device.getVendorId()
    except Exception:
        return len(KNOWN_USB_IDS)
    for index, (known_vid, _pid) in enumerate(KNOWN_USB_IDS):
        if vid == known_vid:
            return index
    return len(KNOWN_USB_IDS)


def open_android_link(port: Optional[str] = None, baud: int = DEFAULT_BAUD, index: int = 0) -> PortLink:
    """Open a USB-TTL adapter on Android, asking for permission if needed.

    index selects which attached adapter when there is more than one (a
    multi-sensor setup via a USB hub): 0 is the single-sensor default and
    keeps today's best-effort behaviour (it'll even take an unrecognised
    device if that's all there is). index > 0 is stricter -- it only counts
    devices that match a known USB-serial bridge VID, since picking "the 2nd
    random USB device" with no such filter is not a safe way to find a second
    radar adapter.
    """
    try:
        from usb4a import usb
        from usbserial4a import serial4a
    except ImportError as exc:  # pragma: no cover - only on device
        raise SerialLinkError("usb4a/usbserial4a not available: %s" % exc)
    _patch_ch34x_baudrate()

    devices = usb.get_usb_device_list()
    if not devices:
        raise SerialLinkError("no USB device attached - check the OTG adapter")

    device = None
    if port:
        for candidate in devices:
            if candidate.getDeviceName() == port:
                device = candidate
                break
        if device is None:
            raise SerialLinkError("USB device %s not attached" % port)
    else:
        ranked = sorted(devices, key=_rank_android_device)
        if index == 0:
            device = ranked[0]
        else:
            known = [d for d in ranked if _rank_android_device(d) < len(KNOWN_USB_IDS)]
            if index >= len(known):
                raise SerialLinkError(
                    "no USB-TTL adapter at index %d (found %d recognised bridge(s) "
                    "of %d USB device(s) attached - check the hub)"
                    % (index, len(known), len(devices))
                )
            device = known[index]

    name = device.getDeviceName()

    if not usb.has_usb_permission(device):
        # Fires the system dialog; the user has to tap Allow. Poll rather than
        # block forever, so a declined dialog surfaces as a normal error.
        usb.request_usb_permission(device)
        deadline = time.time() + PERMISSION_TIMEOUT
        while time.time() < deadline:
            if usb.has_usb_permission(device):
                break
            time.sleep(0.25)
        else:
            raise SerialLinkError("USB permission not granted for %s" % name)

    serial_port = serial4a.get_serial_port(
        name,
        baud,
        DEFAULT_BYTESIZE,
        DEFAULT_PARITY,
        DEFAULT_STOPBITS,
        timeout=READ_TIMEOUT,
    )
    if serial_port is None or not serial_port.is_open:
        raise SerialLinkError("could not open %s at %d baud" % (name, baud))

    try:
        vid, pid = device.getVendorId(), device.getProductId()
        description = "USB %04X:%04X @ %d" % (vid, pid, baud)
    except Exception:
        description = "%s @ %d" % (name, baud)
    files_dir = _android_files_dir()
    capture_path = os.path.join(files_dir, RAW_CAPTURE_FILE) if files_dir and index == 0 else None
    return PortLink(serial_port, description, capture_path=capture_path)


# ---------------------------------------------------------------------------
# Desktop: pyserial
# ---------------------------------------------------------------------------


def _desktop_ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return list(list_ports.comports())


def list_matching_desktop_ports() -> list:
    """All attached ports that look like a known USB-serial bridge, best
    match first, stable-sorted by device name within the same bridge type.

    Deliberately strict: machines are full of virtual COM ports (Bluetooth
    especially) and opening one of those would just hang on an empty stream.
    Only a recognised USB-serial bridge counts.
    """
    candidates = []
    for info in _desktop_ports():
        vid = getattr(info, "vid", None)
        for index, (known_vid, _pid) in enumerate(KNOWN_USB_IDS):
            if vid == known_vid:
                candidates.append((index, info.device))
                break
    candidates.sort()
    return [device for _rank, device in candidates]


def _autodetect_desktop_port() -> Optional[str]:
    ports = list_matching_desktop_ports()
    return ports[0] if ports else None


def open_desktop_link(port: Optional[str] = None, baud: int = DEFAULT_BAUD, index: int = 0) -> PortLink:
    """Open the adapter with pyserial (Windows/Linux/macOS testing).

    index selects which attached adapter when there is more than one
    (a multi-sensor setup); ignored when port is given explicitly.
    """
    try:
        import serial
    except ImportError as exc:
        raise SerialLinkError("pyserial is not installed (pip install pyserial): %s" % exc)

    if port:
        target = port
    else:
        candidates = list_matching_desktop_ports()
        if index >= len(candidates):
            raise SerialLinkError(
                "no USB-TTL adapter at index %d (found %d: %s) - pass --port COM5"
                % (index, len(candidates), ", ".join(candidates) or "none")
            )
        target = candidates[index]

    try:
        serial_port = serial.Serial(
            port=target,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=READ_TIMEOUT,
        )
    except Exception as exc:
        raise SerialLinkError("%s: %s" % (target, exc))

    return PortLink(serial_port, "%s @ %d" % (target, baud))


def open_link(
    port: Optional[str] = None,
    baud: int = DEFAULT_BAUD,
    mock=False,
    index: int = 0,
    mock_phase_offset: float = 0.0,
) -> object:
    """Open the right link for this platform. Raises SerialLinkError on failure.

    mock is False for real hardware, or which sensor to simulate:
    SENSOR_LD2410C / SENSOR_LD2450 (plain True still means the LD2410C).
    """
    if mock == SENSOR_LD2450:
        description = "MOCK LD2450 (no hardware)" if index == 0 else "MOCK LD2450 %d (no hardware)" % (index + 1)
        return MockLD2450Link(description=description, phase_offset=mock_phase_offset)
    if mock:
        description = "MOCK SENSOR (no hardware)" if index == 0 else "MOCK SENSOR %d (no hardware)" % (index + 1)
        return MockLink(description=description, phase_offset=mock_phase_offset)
    if is_android():
        return open_android_link(port=port, baud=baud, index=index)
    return open_desktop_link(port=port, baud=baud, index=index)


def make_link_factory(
    port: Optional[str] = None,
    baud: int = DEFAULT_BAUD,
    mock=False,
    index: int = 0,
    mock_phase_offset: float = 0.0,
) -> Callable[[], object]:
    """Zero-argument opener for LD2410Reader, which reopens on reconnect."""

    def factory() -> object:
        return open_link(port=port, baud=baud, mock=mock, index=index, mock_phase_offset=mock_phase_offset)

    return factory


def describe_backend(mock=False) -> str:
    if mock == SENSOR_LD2450:
        return "MOCK 2450"
    if mock:
        return "MOCK 2410"
    return "ANDROID USB" if is_android() else "PYSERIAL"
