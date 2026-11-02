# LD2410C / LD2450 Tactical Readout

Minimal dark-mode mmWave presence display, read over a USB-to-TTL (CH340)
Type-C adapter. Kivy UI, Android or desktop. Shows targets on a radar-style
scope: range rings for scale, each configured sensor's real bearing as a
line, and a glowing blip with a short fading trail for every detection.

```
ld2410.py               LD2410C data protocol: frame parser + background reader thread
ld2450.py               LD2450 data protocol: frame parser (see "Two sensor types" below)
serial_backend.py       one serial API over Android usb4a, desktop pyserial, and a mock sensor
sensor_array.py         multi-sensor config: name + bearing (+ optional port) per sensor
radar_scope.py          the Kivy scope widget: rings, bearing lines, blips, trails
fieldlog.py             rotating log of state transitions + connection status
main.py                 Kivy "tactical dark" UI
inspect_serial.py       raw serial hexdump + verdict -- run this first against new hardware
test_parser.py          34 LD2410C data-protocol checks: python test_parser.py
test_ld2450_parser.py   34 LD2450 protocol checks: python test_ld2450_parser.py
sensor_detect.py        tells LD2410C and LD2450 apart from the frames themselves
test_sensor_detect.py   19 sensor auto-detection checks: python test_sensor_detect.py
buildozer.spec          Android packaging (see "Android build" below -- the version pins matter)
device_filter.xml       USB vendor ids the APK claims
usb_intent_filter.xml   auto-launch on adapter attach
```

## Two sensor types

**LD2410C** -- distance + moving/stationary state, one fixed beam. No
bearing; you tell the app where it's mounted (see "Multi-sensor" below) and
the scope draws that as a configured line, not a measurement.

**LD2450** -- multiple receive antennas, reports real X/Y position (so real
distance *and* real angle) for up to 3 targets at once, ±60° in front of it.
Wired into the UI and verified on real hardware (PC and phone, same
256000 baud CH34x settings as the LD2410C). The wire protocol was built
from community reverse-engineering, not a datasheet, and one detail -- the
X sign convention (is negative X really left?) -- still needs a real-
hardware check. Stand clearly to one side of the sensor and run:

```bash
python inspect_serial.py --port COM5          # confirm it's really binary, not ASCII
python -c "from ld2450 import verify_sign_convention as v; print(v(x_mm=-842, known_direction='left'))"
```

`inspect_serial.py` dumps raw bytes as hex + ASCII and tells you whether
they look like LD2410C frames, LD2450 frames, or neither -- run it against
*any* new sensor on this project before trusting a parser's assumptions.

## Wiring

| Sensor  | CH340 adapter |
|---------|---------------|
| VCC     | 5V            |
| GND     | GND           |
| TX      | RX            |
| RX      | TX            |

Both sensors use the same link settings: **256000 baud, 8N1**. They stream
frames on their own; no command has to be sent to start either one.

## Run

```bash
python test_parser.py                                # LD2410C protocol tests
python test_ld2450_parser.py                          # LD2450 protocol tests
python test_sensor_detect.py                          # sensor auto-detection tests
python main.py --mock                                 # UI, one synthetic LD2410C
python main.py --mock ld2450                          # UI, one synthetic LD2450
python main.py --port COM5                            # UI, one real LD2410C or LD2450
python main.py --mock --sensors "FRONT:0,REAR:180"    # UI, two synthetic sensors
python main.py --log                                  # UI + field log under ./logs
```

With no `--port`, the first CH340/CP210x/FTDI/PL2303 port found is used. On
Android each USB device is located, permission-requested and opened
automatically -- `--port` is not needed there, and a USB hub is what lets
more than one adapter be attached at once.

The sensor type is detected from the frames themselves (`sensor_detect.py`):
an LD2410C draws on its configured bearing line, an LD2450 draws up to 3
targets at their real positions inside its +/-60 deg field of view.

Tapping the backend label (top right of the app) cycles real hardware ->
synthetic LD2410C -> synthetic LD2450 at runtime -- the installed APK has no command line to
pass `--mock` to, so this is how a demo runs on a phone with nothing wired
up yet, and how the whole UI pipeline gets exercised the moment the app is
installed.

Desktop deps: `pip install -r requirements.txt`. **Kivy has no wheel for
Python 3.14**, so use 3.12 or 3.13 for local UI testing. The parser/protocol
modules and their tests work on any Python 3.8+, no dependencies.

## Multi-sensor setup

```
--sensors "NAME:BEARING[:PORT],NAME:BEARING[:PORT],..."
--sensors "FRONT:0,REAR:180"                # auto-detect ports by order
--sensors "FRONT:0:COM5,REAR:180:COM7"      # pin each sensor to a port
```

Bearing is degrees, 0 = straight ahead, clockwise (90 = right, 180 = behind,
270 = left) -- relative to how *you* mount each sensor, not compass north.
A single LD2410C cannot tell you direction on its own (see `sensor_array.py`
and `radar_scope.py`'s docstrings); this is how you get real per-direction
data out of several of them, each watching its own fixed line. The scope
only ever draws a line at a bearing you actually configured -- never a full
circle implying coverage that isn't real.

Running two sensors on one phone needs a USB-C OTG hub (so both adapters
can be attached at once) plus a second CH340 adapter -- the project's parts
list currently has one adapter for two sensors, which is enough to test
each sensor in turn but not both live simultaneously.

## Android build

```bash
source ~/bz-venv/bin/activate   # a real venv, not --break-system-packages (see below)
cd ~/radarApp                   # build on native Linux filesystem, not /mnt/c (see below)
buildozer -v android debug
```

On Windows this means WSL2 (`wsl --install -d Ubuntu`) -- buildozer does not
run natively on Windows. **This is not a casual `pip install buildozer &&
buildozer android debug`**: getting a clean build required pinning the
whole toolchain (python-for-android release, target Python version, NDK
version, CPU architecture) to the era Kivy 2.3.0 was actually built against,
after hitting six distinct real version-skew bugs building against today's
defaults -- they're all explained where they're pinned, in `buildozer.spec`'s
comments. Two environment gotchas on top of what's in the spec file:

- **Use a real venv for buildozer itself**, not
  `pip install --break-system-packages`. This Ubuntu enforces PEP 668, and
  while that flag gets *your own* `pip install buildozer` past it,
  `python-for-android`'s own internal `pip install` subprocess calls don't
  pass that flag themselves -- they fail unless buildozer is already running
  from a venv's interpreter in the first place.
- **Build from the native Linux filesystem** (e.g. `~/radarApp`), not
  `/mnt/c/...` -- copy the source over first. `python-for-android` needs
  case-sensitive paths and proper symlink support that Windows-mounted
  paths (9p/drvfs) don't reliably give it.

Once it succeeds, the APK lands in `bin/`. Skip `adb install` entirely if
this project folder lives under OneDrive (as it does by default): copy the
APK there and it syncs automatically -- open it from the OneDrive app on the
phone and tap to install (Android will prompt to allow installs from that
app, normal for a sideloaded APK).

Other notes:

- Two debug hooks live in the app's private files dir (reach it with
  `adb shell run-as org.tactical.ld2410tactical`, restart the app after
  changing them): `ch34x_baud.json` (`{"reg1312": ..., "reg0f2c": ...}`)
  forces the CH34x baud registers, and an empty `raw_capture.enable` makes
  the app dump the first 8 KB of each connection to `raw_capture.bin`.
- `android.add_resources` in `buildozer.spec` needs buildozer >= 1.5. On an
  older version, drop `device_filter.xml` into
  `.buildozer/android/platform/build-*/dists/*/src/main/res/xml/` before the
  packaging step, or remove the `meta-data` line from `usb_intent_filter.xml`
  (you then just lose auto-launch; the permission dialog still works).
- USB host mode is a hardware feature, not a runtime permission. Access is
  granted per device by the system dialog `usb4a` raises on first connect; the
  app polls for up to 20 s and reports a refusal as a normal error.
- `android.features` for USB host is deliberately *not* set in
  `buildozer.spec` -- it only affects Play Store device filtering (irrelevant
  for a sideloaded APK) and the pinned buildozer/p4a versions don't agree on
  that CLI flag's spelling. See the comment there if store distribution ever
  matters.

## Reading the UI

- **Status line** — each configured sensor's link state: `Hz` figure
  (green) when frames are arriving, `---` (amber) when the port is open but
  the sensor is silent, `ERR` (red) with the reason, `OFF` (grey) when
  disconnected. A single-sensor setup shows this with no sensor-name prefix.
- **Scope** — range rings for distance scale (a neutral ruler, same in every
  direction), a line for each sensor's real configured bearing (nothing
  elsewhere implies coverage), and a blip per detection: **red** for moving,
  **amber** for stationary, with a short fading trail so movement toward or
  away from the sensor is visible, not just a jumping number.
- **Big number** — distance in centimetres for whichever target is closest
  right now (tagged with its sensor's name in a multi-sensor setup), red
  while held, dim grey and `---` when clear.
- **Bottom line** — each sensor's exact moving/stationary distance and
  energy, plus total frames decoded and dropped.

The reader reconnects by itself every 2 s per sensor after an error or an
unplug, so the app can be left running while an adapter is pulled and
reseated.

## Field log

Written automatically on Android; pass `--log` on desktop (or `--no-log` to
silence it on Android). Logs state *transitions* (room goes from clear to
occupied and back) plus a heartbeat every 5 s while a target is held, not
every frame -- at 15 Hz that would be thousands of lines a minute for no
benefit. Also logs every connection status change. One file per run,
rotated at 2 MB x 5 backups, under `./logs` (desktop) or the app's private
storage (Android).

## Design notes

- **Nothing blocks the UI thread.** Each sensor's `LD2410Reader` owns a
  daemon thread that does all serial reads; its callbacks only store the
  newest frame in an attribute. The Kivy clock redraws from those attributes
  at 20 Hz, decoupled from the sensors' own frame rate.
- **The LD2410C parser is length-driven, not size-driven.** It searches for
  `F4 F3 F2 F1`, reads the little-endian length field, waits for
  `6 + length + 4` bytes, then requires `F8 F7 F6 F5` at the end plus `AA`/`55`
  around the payload. A frame that fails any check costs one discarded byte and
  the search resumes, so a mid-stream glitch loses one frame rather than
  desynchronising the link. The buffer is capped at 4 KB. `ld2450.py`'s
  parser follows the same shape against its own sync bytes.
- **Byte 18 of an LD2410C frame is not a real checksum.** The spec calls it
  a check byte, and this sensor always sends `00`; validating it would only
  reject valid frames, so the four structural markers do the integrity work
  instead.
- **LD2410C engineering-mode frames (data type `01`) are accepted too.**
  Their payload is longer but the first 11 bytes are the same basic target
  data, so switching the sensor into engineering mode will not blank the
  display.
- **A detection is held for 350 ms** after the last frame reporting it. The
  sensor flickers between states at 15 Hz and the display would otherwise
  strobe; raw per-frame values are still available on the callback.
- **The scope never draws coverage that isn't configured.** No rotating
  sweep (it would be purely decorative -- nothing here is scanned), no
  generic crosshair implying four-way coverage. Only the bearings passed to
  `configure_sensors()` get a line; everything else honestly shows nothing.
- **Canvas gotcha worth knowing if you touch `radar_scope.py`:** draw into
  `self.canvas.before`, never `self.canvas` directly. Kivy's `add_widget()`
  links a child widget's canvas into the parent's `self.canvas`; a plain
  `self.canvas.clear()` rips those links out too, leaving child Labels (the
  ring/bearing text) valid but permanently undrawn after the first redraw.
