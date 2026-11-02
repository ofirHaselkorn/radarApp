"""Tactical dark-mode readout for the HLK-LD2410C / LD2450 mmWave radars.

    python main.py --mock                              # one synthetic LD2410C
    python main.py --mock ld2450                        # one synthetic LD2450
    python main.py --port COM5                          # one real sensor
    python main.py --mock --sensors "FRONT:0,REAR:180"   # two synthetic sensors
    (on Android each USB device is found and opened automatically)

Which sensor is on each link is detected from its frames (sensor_detect.py),
so the same app drives either one with no setting to change. An LD2410C
reports distance only, never bearing -- see sensor_array.py -- so it is
plotted at the fixed bearing you gave it. An LD2450 reports a real X/Y
position for up to 3 targets, plotted where they actually are, rotated by
that same mounting bearing. With no --sensors flag, the app runs a single
sensor named "SENSOR" at bearing 0, driven by --port/--mock as before.

Tapping the backend label (top right) cycles real hardware -> synthetic
LD2410C -> synthetic LD2450 -> real hardware at runtime -- the installed APK
has no command line to pass --mock to, so this is how a demo is run on a
phone with no sensor attached, and how the UI pipeline gets exercised the
moment the app is installed.

A field log (state transitions + connection status, not every frame) is
written automatically on Android; pass --log on desktop to get the same
file under ./logs, or --no-log to silence it on Android.

The serial work happens on a background thread per sensor, inside
LD2410Reader. Its callbacks only drop the newest frame into an attribute;
the UI redraws itself from those attributes on a Kivy clock tick, so the
main thread never waits on serial I/O.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List, Optional

# Must be set before kivy is imported, otherwise Kivy claims sys.argv and
# chokes on our own flags (--port, --mock).
os.environ.setdefault("KIVY_NO_ARGS", "1")

from kivy.app import App  # noqa: E402
from kivy.clock import Clock  # noqa: E402
from kivy.core.window import Window  # noqa: E402
from kivy.factory import Factory  # noqa: E402
from kivy.lang import Builder  # noqa: E402
from kivy.properties import ListProperty, StringProperty  # noqa: E402
from kivy.uix.boxlayout import BoxLayout  # noqa: E402

import ld2410  # noqa: E402
from fieldlog import FieldLogger, FrameRecorder  # noqa: E402
from ld2410 import LD2410Reader, TargetFrame  # noqa: E402
from ld2450 import TargetSmoother, TrajectoryFrame  # noqa: E402
from radar_scope import RadarScope  # noqa: E402
from sensor_array import SensorConfig, parse_sensor_spec  # noqa: E402
from sensor_detect import (  # noqa: E402
    FIELD_OF_VIEW_HALF_DEG,
    SENSOR_LD2410C,
    SENSOR_LD2450,
    AutoFrameParser,
)
from serial_backend import (  # noqa: E402
    DEFAULT_BAUD,
    describe_backend,
    is_android,
    make_link_factory,
)

Factory.register("RadarScope", cls=RadarScope)

# -- palette: black field, one hot colour, everything else barely there ------
BLACK = (0, 0, 0, 1)
DIMMER = (0.16, 0.18, 0.16, 1)       # labels, chrome
RED = (0.88, 0.07, 0.07, 1)          # error
AMBER = (0.80, 0.52, 0.04, 1)        # connecting / stale
GREEN = (0.11, 0.62, 0.22, 1)        # link up
GREY = (0.45, 0.48, 0.45, 1)

# Seconds without a frame before a sensor's link is called dead.
LINK_TIMEOUT = 1.2
UI_REFRESH_HZ = 30.0
# What tapping the backend label steps through: real, then each mock sensor.
DEMO_MODES = (False, SENSOR_LD2410C, SENSOR_LD2450)
# Mock sensors are staggered this many seconds apart so a multi-sensor demo
# doesn't show every sensor doing the exact same thing in lockstep.
MOCK_PHASE_STEP = 11.0

KV = """
#:import dp kivy.metrics.dp

<RadarRoot>:
    orientation: 'vertical'
    padding: dp(16)
    spacing: dp(6)
    canvas.before:
        Color:
            rgba: 0, 0, 0, 1
        Rectangle:
            pos: self.pos
            size: self.size

    BoxLayout:
        size_hint_y: None
        height: dp(24)
        Label:
            text: root.sensor_text
            bold: True
            font_size: dp(15)
            color: root.chrome_color
            halign: 'left'
            valign: 'middle'
            text_size: self.size
        Button:
            size_hint_x: None
            width: dp(84)
            text: root.rec_text
            font_size: dp(12)
            bold: True
            background_normal: ''
            background_down: ''
            background_color: 0, 0, 0, 0
            color: root.rec_color
            on_release: app.toggle_recording()
        Button:
            text: root.backend_name
            font_size: dp(12)
            bold: True
            halign: 'right'
            valign: 'middle'
            text_size: self.size
            background_normal: ''
            background_down: ''
            background_color: 0, 0, 0, 0
            color: root.chrome_color
            on_release: app.toggle_demo_mode()

    Label:
        id: status_label
        size_hint_y: None
        height: dp(18)
        text: root.status_text
        color: root.status_color
        font_size: dp(12)
        bold: True
        halign: 'left'
        valign: 'middle'
        text_size: self.size

    AnchorLayout:
        size_hint_y: 1
        RadarScope:
            id: scope
            size_hint: None, None
            size: (min(self.parent.width, self.parent.height) - dp(8),) * 2

    Label:
        size_hint_y: None
        height: dp(16)
        text: root.targets_text
        color: 0.55, 0.60, 0.55, 1
        font_size: dp(11)
        bold: True
        shorten: True
        halign: 'center'
        valign: 'middle'
        text_size: self.size

    Label:
        size_hint_y: None
        height: dp(14)
        text: root.telemetry_text
        color: root.chrome_color
        font_size: dp(10)
        shorten: True
        halign: 'center'
        valign: 'middle'
        text_size: self.size
"""


class RadarRoot(BoxLayout):
    """Pure view: every label reads one of these properties."""

    backend_name = StringProperty("")
    sensor_text = StringProperty("RADAR")
    targets_text = StringProperty("")
    rec_text = StringProperty("REC")
    rec_color = ListProperty([0.40, 0.42, 0.40, 1])
    status_text = StringProperty("STANDBY")
    status_color = ListProperty(list(GREY))
    chrome_color = ListProperty(list(DIMMER))
    telemetry_text = StringProperty("")


class _SensorTrack:
    """Runtime state for one configured sensor: its reader, its latest
    frame, and the small bookkeeping _tick needs. Plain data holder -- all
    the behaviour lives in RadarApp, which owns a list of these."""

    def __init__(self, config: SensorConfig, index: int):
        self.config = config
        self.index = index
        self.reader: Optional[LD2410Reader] = None
        self.frame: Optional[object] = None  # TargetFrame or TrajectoryFrame
        # LD2450 display smoothing; smoothed_frame is the last frame fed to it.
        self.smoother = TargetSmoother()
        self.smoothed_frame: Optional[object] = None
        # Detected sensor type (sensor_detect.SENSOR_*), None until frames lock on.
        self.kind: Optional[str] = None
        self.status = (ld2410.STATUS_STOPPED, "standby")
        self.frame_count = 0
        self.last_frame_time = 0.0
        self.rate = 0.0
        self.rate_mark = time.time()
        self.rate_count = 0


class RadarApp(App):
    def __init__(self, sensors: List[SensorConfig], baud: int = DEFAULT_BAUD,
                 mock=False, log_enabled: bool = False, **kwargs):
        super().__init__(**kwargs)
        self.title = "RADAR TACTICAL"
        self._baud = baud
        # False for real hardware, else which sensor to simulate. Mutable at
        # runtime: tapping the backend label (top right) cycles it, so a
        # phone with no sensor attached can still demo the UI.
        self._mock = mock
        self._log_enabled = log_enabled
        self._field_log: Optional[FieldLogger] = None
        self._recorder: Optional[FrameRecorder] = None
        self.root_widget: Optional[RadarRoot] = None
        self._tracks: List[_SensorTrack] = [
            _SensorTrack(config, i) for i, config in enumerate(sensors)
        ]

    # -- app lifecycle ----------------------------------------------------
    def build(self):
        Builder.load_string(KV)
        Window.clearcolor = BLACK
        try:
            # Keep the screen on: a readout that sleeps is useless in the field.
            Window.allow_screensaver = False
        except Exception:
            pass

        self.root_widget = RadarRoot()
        self.root_widget.ids.scope.configure_sensors(
            [(t.config.name, t.config.bearing_deg) for t in self._tracks]
        )

        if self._log_enabled:
            try:
                self._field_log = FieldLogger()
            except Exception:
                # A log directory that can't be created must never take the
                # app down with it; the readout works fine without one.
                self._field_log = None
        self._update_chrome_text()

        Clock.schedule_interval(self._tick, 1.0 / UI_REFRESH_HZ)
        # Connect a moment after the first frame is drawn, so a slow USB
        # permission dialog never delays the window appearing.
        Clock.schedule_once(lambda _dt: self.connect(), 0.4)
        return self.root_widget

    def on_stop(self):
        self.disconnect()
        self._stop_recording()
        if self._field_log is not None:
            self._field_log.close()
            self._field_log = None

    def _update_chrome_text(self) -> None:
        if self.root_widget is None:
            return
        name = describe_backend(self._mock)
        if len(self._tracks) > 1:
            name += " x%d" % len(self._tracks)
        self.root_widget.backend_name = "%s · LOG" % name if self._field_log else name

    # -- connection -------------------------------------------------------
    def connect(self) -> None:
        for track in self._tracks:
            if track.reader is not None and track.reader.is_running:
                continue
            track.frame = None
            track.status = (ld2410.STATUS_CONNECTING, "opening serial port")
            track.reader = LD2410Reader(
                open_link=make_link_factory(
                    track.config.port, self._baud, self._mock,
                    index=track.index, mock_phase_offset=track.index * MOCK_PHASE_STEP,
                ),
                on_frame=lambda frame, t=track: self._on_frame(t, frame),
                on_status=lambda state, detail, t=track: self._on_status(t, state, detail),
                stale_after=LINK_TIMEOUT,
                parser_factory=AutoFrameParser,
            )
            track.reader.start()

    def disconnect(self) -> None:
        for track in self._tracks:
            reader, track.reader = track.reader, None
            if reader is not None:
                reader.stop()
            track.frame = None
            track.kind = None
            track.smoother.reset()
            track.status = (ld2410.STATUS_STOPPED, "disconnected")
        if self.root_widget is not None:
            self.root_widget.ids.scope.reset()

    def toggle_demo_mode(self) -> None:
        """Step to the next of real / mock LD2410C / mock LD2450.

        Lets a phone with no sensor attached still show live-looking data --
        there is no command line on an installed APK to pass --mock to.
        """
        current = DEMO_MODES.index(self._mock) if self._mock in DEMO_MODES else 0
        self._mock = DEMO_MODES[(current + 1) % len(DEMO_MODES)]
        self._update_chrome_text()
        was_running = any(t.reader is not None and t.reader.is_running for t in self._tracks)
        self.disconnect()
        if was_running:
            self.connect()

    # -- recording ---------------------------------------------------------
    def toggle_recording(self) -> None:
        """Save every frame to a CSV until tapped again (for tuning)."""
        if self._recorder is not None:
            self._stop_recording()
            return
        try:
            self._recorder = FrameRecorder()
        except Exception:
            self._recorder = None  # storage trouble must not take the app down

    def _stop_recording(self) -> None:
        recorder, self._recorder = self._recorder, None
        if recorder is not None:
            recorder.close()

    # -- reader thread callbacks (keep these cheap, no UI access) ---------
    def _on_frame(self, track: _SensorTrack, frame) -> None:
        track.frame = frame
        track.frame_count += 1
        track.rate_count += 1
        track.last_frame_time = frame.timestamp
        if self._field_log is not None:
            self._field_log.on_frame(frame, sensor=track.config.name)
        recorder = self._recorder
        if recorder is not None:
            recorder.on_frame(frame, sensor=track.config.name)

    def _on_status(self, track: _SensorTrack, state: str, detail: str) -> None:
        track.status = (state, detail)
        if self._field_log is not None:
            self._field_log.on_status(state, detail, sensor=track.config.name)

    # -- UI refresh (main thread) -----------------------------------------
    def _tick(self, _dt: float) -> None:
        # _dt (seconds since the last tick) isn't used -- Kivy's Clock always
        # passes it, kept only so this matches schedule_interval's callback
        # signature.
        root = self.root_widget
        if root is None:
            return

        now = time.time()
        scope = root.ids.scope
        multi = len(self._tracks) > 1

        status_parts = []
        target_parts = []
        worst_severity, worst_color = -1, GREY

        for track in self._tracks:
            if now - track.rate_mark >= 1.0:
                track.rate = track.rate_count / (now - track.rate_mark)
                track.rate_mark = now
                track.rate_count = 0

            state, detail = track.status
            running = track.reader is not None and track.reader.is_running
            link_up = running and (now - track.last_frame_time) < LINK_TIMEOUT

            if not running:
                phrase, severity, color = "%s OFF" % track.config.name, 0, GREY
            elif state == ld2410.STATUS_ERROR:
                phrase, severity, color = "%s ERR" % track.config.name, 3, RED
            elif not link_up:
                phrase, severity, color = "%s ---" % track.config.name, 2, AMBER
            else:
                phrase, severity, color = "%s %.0fHz" % (track.config.name, track.rate), 1, GREEN
            status_parts.append(phrase if multi else phrase.split(" ", 1)[1])
            if severity > worst_severity:
                worst_severity, worst_color = severity, color

            kind = track.reader.parser.kind if track.reader is not None else None
            if kind != track.kind:
                track.kind = kind
                track.smoother.reset()
                scope.reset(track.config.name)
                scope.set_field_of_view(track.config.name, FIELD_OF_VIEW_HALF_DEG.get(kind, 0.0))

            frame = track.frame
            if link_up and isinstance(frame, TrajectoryFrame):
                if frame is not track.smoothed_frame:
                    track.smoother.update(frame)
                    track.smoothed_frame = frame
                active = track.smoother.targets(now)
                scope.plot_targets(
                    track.config.name,
                    [(t.slot, t.distance_cm, t.bearing_deg, t.speed_cms != 0, t.echo) for t in active],
                    now=now,
                )
                prefix = "%s " % track.config.name if multi else ""
                # "?" marks a target the sensor just lost (last known position),
                # "~" one that is probably a reflection of a nearer target.
                target_parts += [
                    "%sT%d %.1fm %+.0f°%s" % (prefix, t.slot, t.distance_cm / 100.0, t.bearing_deg,
                                              "?" if t.held else "~" if t.echo else "")
                    for t in active
                ]
            elif link_up and isinstance(frame, TargetFrame):
                scope.plot(
                    track.config.name,
                    frame.moving_distance_cm if frame.has_moving else None, frame.moving_energy,
                    frame.stationary_distance_cm if frame.has_stationary else None, frame.stationary_energy,
                    now=frame.timestamp,
                )

        scope.refresh(now)

        kinds = sorted(set(t.kind for t in self._tracks if t.kind))
        root.sensor_text = " + ".join(kinds) if kinds else "RADAR"
        root.targets_text = "   ".join(target_parts)
        recorder = self._recorder
        if recorder is not None:
            root.rec_text = "REC %d" % recorder.frames
            root.rec_color = list(RED)
        else:
            root.rec_text = "REC"
            root.rec_color = [0.40, 0.42, 0.40, 1]

        root.status_text = "   ".join(status_parts) if status_parts else "STANDBY"
        root.status_color = list(worst_color)

        total_frames = sum(t.frame_count for t in self._tracks)
        total_dropped = sum(
            t.reader.parser.frames_rejected for t in self._tracks if t.reader is not None
        )
        total_junk = sum(
            t.reader.parser.bytes_discarded for t in self._tracks if t.reader is not None
        )
        # On a phone there's no console and the field log is in app-private
        # storage, so an error's reason has to be on screen to be seen at all.
        errors = [
            "%s: %s" % (t.config.name, t.status[1]) if multi else t.status[1]
            for t in self._tracks if t.status[0] == ld2410.STATUS_ERROR
        ]
        if errors:
            root.telemetry_text = "   ".join(errors)
        else:
            # JUNK climbing while FRAMES stays at 0 = bytes arriving but not
            # parsing, i.e. a wrong baud rate or a wrong sensor type.
            root.telemetry_text = "FRAMES %d   DROPPED %d   JUNK %d" % (
                total_frames, total_dropped, total_junk)


def _parse_args(argv):
    ap = argparse.ArgumentParser(description="LD2410C / LD2450 tactical readout")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0 (single-sensor mode)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument(
        "--mock", nargs="?", const="ld2410c", choices=("ld2410c", "ld2450"),
        help="synthetic sensor(s), no hardware (default type: ld2410c)",
    )
    ap.add_argument(
        "--sensors",
        help='multi-sensor setup: "NAME:BEARING[:PORT],..." e.g. "FRONT:0,REAR:180" '
             "(bearing in degrees, 0 = forward, clockwise)",
    )
    ap.add_argument("--log", action="store_true", help="write a field log (default: on for Android)")
    ap.add_argument("--no-log", action="store_true", help="disable the field log on Android")
    # Tolerate anything else on argv (Kivy flags, Android extras).
    args, _unknown = ap.parse_known_args(argv)
    return args


if __name__ == "__main__":
    opts = _parse_args(sys.argv[1:])
    log_enabled = (not opts.no_log) and (opts.log or is_android())

    if opts.sensors:
        try:
            sensor_configs = parse_sensor_spec(opts.sensors)
        except ValueError as exc:
            print("error: %s" % exc, file=sys.stderr)
            sys.exit(2)
    else:
        sensor_configs = [SensorConfig(name="SENSOR", bearing_deg=0.0, port=opts.port)]

    mock_kind = {None: False, "ld2410c": SENSOR_LD2410C, "ld2450": SENSOR_LD2450}[opts.mock]
    RadarApp(
        sensors=sensor_configs, baud=opts.baud, mock=mock_kind, log_enabled=log_enabled
    ).run()
