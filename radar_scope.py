"""Radar-style scope widget: range rings and a blip for each configured
sensor's moving/stationary target, each with a short fading trail so a
target walking in or out leaves a comet rather than jumping frame to frame.

A single LD2410C reports distance only, not bearing -- it is one fixed beam,
not a scanning array. This widget never invents an angle: each blip is
plotted at the fixed bearing *you* configured for that physical sensor via
configure_sensors(). With one sensor, that is one fixed line from centre.
With several sensors mounted in different directions, each gets its own
real line -- true per-sensor data, not a guess. There is no rotating sweep
line -- it would be purely decorative (nothing here is scanned), so rather
than show motion that means nothing, the scope just doesn't have it.

The face itself is sized to match: it's the smallest circular sector (pie
slice) that contains every configured bearing, not an automatic full circle.
One forward-facing sensor draws a half-circle "field of view" shape, not a
full disc it can't possibly fill on its own; a front+rear pair still draws
a half-circle too (two bearings 180 deg apart sit exactly at a half-circle's
two straight edges -- see _minimal_sector()). Only bearings spread across
more than half the compass genuinely need more arc, and bearings surrounding
you on all sides are the only case that earns the full circle.

An LD2450 is different: it reports each target's own X/Y position, so its
blips come in through plot_targets() with a real per-target bearing (offset
by the sensor's mounting bearing), and set_field_of_view() swaps its single
bearing line for the two edges of the wedge it can actually see.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Deque, Dict, Iterable, NamedTuple, Optional, Tuple

from kivy.graphics import Color, Ellipse, Line
from kivy.properties import NumericProperty
from kivy.uix.label import Label
from kivy.uix.widget import Widget

METERS_PER_RING_CM = 100.0    # one ring per metre, so the base scale numbers mean exactly what they say
TRAIL_FADE_SECONDS = 0.6      # how long a blip's afterglow trail persists (longer reads as lag)
MIN_SECTOR_SPAN_DEG = 180.0   # a lone sensor still gets a half-circle, not a sliver
SECTOR_MARGIN_DEG = 15.0      # pad past the exact bearings so edge blips aren't flush against the cut

FACE_COLOR = (0.02, 0.05, 0.03, 1)
GRID_COLOR = (0.10, 0.28, 0.14, 1)
COVERAGE_COLOR = (0.30, 0.34, 0.30, 1)    # a configured sensor's actual watched line
BEARING_LABEL_COLOR = (0.55, 0.60, 0.55, 1)
SCALE_LABEL_COLOR = (0.40, 0.80, 0.44, 1)
MOVING_COLOR = (0.95, 0.15, 0.15)         # red
STATIONARY_COLOR = (0.90, 0.60, 0.05)     # amber
OWNSHIP_COLOR = (0.55, 0.60, 0.55, 1)


XY_TARGET_ENERGY = 70          # the LD2450 reports no energy; a fixed mid-size blip

class TrailPoint(NamedTuple):
    distance_cm: float
    energy: int
    t: float
    # Set only for targets with a reported position (LD2450): the absolute
    # bearing to draw at, and whether the target was moving. None means
    # "use the sensor's configured bearing and the trail's own colour".
    bearing_deg: Optional[float] = None
    moving: Optional[bool] = None


def _minimal_sector(bearings_deg, min_span: float = MIN_SECTOR_SPAN_DEG, margin: float = SECTOR_MARGIN_DEG):
    """Smallest arc (start_deg, span_deg), in our bearing convention (0 = up,
    clockwise), that contains every given bearing -- padded a little, never
    narrower than min_span, never wider than a full circle.

    Works by finding the single largest *gap* between consecutive bearings
    going around the compass; the rest of the circle (360 - that gap) is the
    minimal arc that has to contain every point, since nothing lives in the
    gap. Two bearings exactly 180 deg apart (e.g. a front+rear sensor pair)
    produce two equal 180 deg gaps -- either choice yields a valid half-circle
    with both bearings sitting exactly on its two straight edges.
    """
    uniq = sorted(set(round(b, 3) % 360.0 for b in bearings_deg))
    if not uniq:
        return -min_span / 2.0, min_span
    if len(uniq) == 1:
        return uniq[0] - min_span / 2.0, min_span

    gaps = []
    for i, a in enumerate(uniq):
        b = uniq[(i + 1) % len(uniq)]
        if i == len(uniq) - 1:
            b += 360.0
        gaps.append((b - a, b))  # (gap width, where the gap ends / arc begins)
    _largest_gap, gap_end = max(gaps)

    natural_span = 360.0 - _largest_gap
    # Center on the natural (unpadded) arc, not on wherever its start
    # happens to land -- otherwise flooring a tight cluster up to min_span
    # anchors the padded sector at the cluster's edge instead of around it,
    # leaving most of the visible arc empty on the far side from the actual
    # sensors.
    center = (gap_end % 360.0) + natural_span / 2.0
    span = min(360.0, max(min_span, natural_span + 2 * margin))
    return center - span / 2.0, span


# Kivy's Ellipse/Line angle_start/angle_end turns out to already use 0 = up,
# clockwise-positive -- the exact same convention as our bearing_deg.
# Verified empirically (rendered isolated quarter-circles at known angle
# pairs and read the result), not trusted from documentation, after an
# earlier version of this code applied a 90-bearing_deg "conversion" on the
# wrong assumption that Kivy uses the standard math convention (0 = east,
# counter-clockwise) -- that rotated every sector 90 degrees off from where
# it should have been. No conversion is needed; see _kivy_sector_angles().


class RadarScope(Widget):
    """A circular radar-style scope face. Call plot()/refresh() once per UI
    frame (see main.py's _tick for the call pattern)."""

    max_range_cm = NumericProperty(600.0)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Keyed by (sensor_name, "moving"|"stationary").
        self._trails: Dict[Tuple[str, str], Deque[TrailPoint]] = {}
        self._bearings: Dict[str, float] = {}
        # Sensors that see a wedge rather than one line: name -> half-angle.
        self._fov_half_deg: Dict[str, float] = {}
        self._bearing_labels: Dict[str, Label] = {}
        # Not a number on every ring (that's chart-axis clutter, not a sonar
        # display) -- a compact 1/2/3/... scale along one straight edge of
        # the sector instead, grown lazily to match however many rings
        # max_range_cm currently needs.
        self._scale_labels: list = []
        self._sector_start_deg = -MIN_SECTOR_SPAN_DEG / 2.0
        self._sector_span_deg = MIN_SECTOR_SPAN_DEG
        self.bind(pos=self._draw, size=self._draw)

    # -- configuration --------------------------------------------------
    def configure_sensors(self, sensors: Iterable[Tuple[str, float]]) -> None:
        """sensors: iterable of (name, bearing_deg). Call once at startup to
        lay down the bearing tick labels (sensors sharing a bearing share
        one label)."""
        for label in self._bearing_labels.values():
            self.remove_widget(label)
        self._bearing_labels.clear()
        self._bearings.clear()
        self._fov_half_deg.clear()

        by_bearing: Dict[float, list] = {}
        for name, bearing_deg in sensors:
            self._bearings[name] = bearing_deg
            by_bearing.setdefault(round(bearing_deg, 1), []).append(name)

        self._update_sector()

        for bearing_deg, names in by_bearing.items():
            label = Label(
                text="/".join(names), font_size="11sp", bold=True,
                color=BEARING_LABEL_COLOR, size_hint=(None, None), size=(100, 16),
            )
            self.add_widget(label)
            self._bearing_labels[",".join(names)] = label
        self._draw()

    def set_field_of_view(self, sensor_name: str, half_angle_deg: float) -> None:
        """Mark a configured sensor as seeing a wedge of +/-half_angle_deg
        around its bearing (0 = back to a single bearing line). Called once
        the app has worked out which sensor type is on that link."""
        if half_angle_deg > 0:
            if self._fov_half_deg.get(sensor_name) == half_angle_deg:
                return
            self._fov_half_deg[sensor_name] = half_angle_deg
        elif self._fov_half_deg.pop(sensor_name, None) is None:
            return
        self._update_sector()
        self._draw()

    def _update_sector(self) -> None:
        bearings = list(self._bearings.values())
        for name, half in self._fov_half_deg.items():
            bearing = self._bearings.get(name, 0.0)
            bearings += [bearing - half, bearing + half]
        self._sector_start_deg, self._sector_span_deg = _minimal_sector(bearings)

    # -- public API -------------------------------------------------------
    def plot(
        self,
        sensor_name: str,
        moving_distance_cm: Optional[float],
        moving_energy: int,
        stationary_distance_cm: Optional[float],
        stationary_energy: int,
        now: Optional[float] = None,
    ) -> None:
        """Record this tick's target positions for one sensor (None for a
        target that is currently absent). Call once per sensor per tick,
        then refresh() once after all sensors are plotted."""
        now = time.time() if now is None else now
        if moving_distance_cm is not None:
            self._trail(sensor_name, "moving").append(TrailPoint(moving_distance_cm, moving_energy, now))
        if stationary_distance_cm is not None:
            self._trail(sensor_name, "stationary").append(
                TrailPoint(stationary_distance_cm, stationary_energy, now)
            )

    def plot_targets(self, sensor_name: str, targets, now: Optional[float] = None) -> None:
        """Record this tick's positioned targets for one sensor (LD2450).

        targets: iterable of (slot, distance_cm, bearing_deg, moving) where
        bearing_deg is relative to the sensor's own facing (0 = straight
        out of it, clockwise positive). Each slot keeps its own trail.
        """
        now = time.time() if now is None else now
        mount = self._bearings.get(sensor_name, 0.0)
        for slot, distance_cm, bearing_deg, moving in targets:
            self._trail(sensor_name, "target%d" % slot).append(
                TrailPoint(distance_cm, XY_TARGET_ENERGY, now, mount + bearing_deg, bool(moving))
            )

    def refresh(self, now: Optional[float] = None) -> None:
        """Age out stale trail points and redraw. Call once per tick, after
        any plot() calls -- including when nothing new came in, so existing
        blips keep fading on schedule."""
        now = time.time() if now is None else now
        for trail in self._trails.values():
            self._prune(trail, now)
        self._draw()

    def reset(self, sensor_name: Optional[str] = None) -> None:
        """Clear trails -- all of them, or just one sensor's -- e.g. on
        disconnect, so a stale blip doesn't linger after the link drops."""
        if sensor_name is None:
            for trail in self._trails.values():
                trail.clear()
        else:
            for (name, _kind), trail in self._trails.items():
                if name == sensor_name:
                    trail.clear()
        self._draw()

    # -- internals ------------------------------------------------------
    def _trail(self, sensor_name: str, kind: str) -> Deque[TrailPoint]:
        key = (sensor_name, kind)
        trail = self._trails.get(key)
        if trail is None:
            trail = deque(maxlen=20)
            self._trails[key] = trail
        return trail

    @staticmethod
    def _prune(trail: Deque[TrailPoint], now: float) -> None:
        while trail and now - trail[0].t > TRAIL_FADE_SECONDS:
            trail.popleft()

    def _ring_count(self) -> int:
        return max(1, round(self.max_range_cm / METERS_PER_RING_CM))

    def _ensure_scale_labels(self, count: int) -> None:
        while len(self._scale_labels) < count:
            label = Label(
                font_size="10sp", bold=True, color=SCALE_LABEL_COLOR,
                size_hint=(None, None), size=(20, 14),
            )
            self.add_widget(label)
            self._scale_labels.append(label)

    def _center_radius(self):
        cx = self.center_x
        cy = self.center_y
        r = max(0.0, min(self.width, self.height) / 2.0 - 4)
        return cx, cy, r

    def _point(self, cx: float, cy: float, r: float, bearing_deg: float, distance_cm: float):
        frac = max(0.0, min(1.0, distance_cm / self.max_range_cm))
        rad = math.radians(bearing_deg)
        return cx + r * frac * math.sin(rad), cy + r * frac * math.cos(rad)

    def _kivy_sector_angles(self):
        """(angle_start, angle_end) for Kivy's Ellipse/Line arc kwargs,
        covering the same sector as self._sector_start_deg/_span_deg. A
        direct pass-through, not a conversion -- see the comment above."""
        return self._sector_start_deg, self._sector_start_deg + self._sector_span_deg

    def _draw(self, *_args) -> None:
        # Draw into canvas.before, not canvas itself: add_widget() links each
        # child label's canvas into self.canvas, and a plain self.canvas.clear()
        # would silently rip those links out along with our own instructions,
        # leaving the labels valid but permanently undrawn after frame one.
        self.canvas.before.clear()
        cx, cy, r = self._center_radius()
        if r <= 0:
            for label in self._bearing_labels.values():
                label.opacity = 0
            for label in self._scale_labels:
                label.opacity = 0
            return

        kivy_start, kivy_end = self._kivy_sector_angles()
        is_full_circle = self._sector_span_deg >= 359.999
        ring_count = self._ring_count()

        with self.canvas.before:
            Color(*FACE_COLOR)
            if is_full_circle:
                Ellipse(pos=(cx - r, cy - r), size=(r * 2, r * 2))
            else:
                Ellipse(pos=(cx - r, cy - r), size=(r * 2, r * 2),
                        angle_start=kivy_start, angle_end=kivy_end)

            # Range rings are a neutral distance scale (same in every
            # direction, like a ruler) -- fine to show full circles for.
            # Coverage is not: no generic N/S/E/W crosshair here, because
            # this sensor setup does not watch every direction. Only the
            # bearings actually configured get a line, drawn next. The rings
            # themselves are clipped to the same sector as the face, not
            # just the face -- a full ring poking out past a half-circle
            # face would look like a rendering bug, not a design choice.
            Color(*GRID_COLOR)
            for i in range(1, ring_count + 1):
                if is_full_circle:
                    Line(circle=(cx, cy, r * i / ring_count), width=1.0)
                else:
                    Line(circle=(cx, cy, r * i / ring_count, kivy_start, kivy_end), width=1.0)
                    # Close the pie-slice edges on the two straight sides of
                    # a partial sector, so it reads as "this far around", not
                    # as a ring that just stops mid-air.
                    for edge_bearing in (self._sector_start_deg, self._sector_start_deg + self._sector_span_deg):
                        edge_rad = math.radians(edge_bearing)
                        inner = r * (i - 1) / ring_count
                        outer = r * i / ring_count
                        Line(points=[
                            cx + inner * math.sin(edge_rad), cy + inner * math.cos(edge_rad),
                            cx + outer * math.sin(edge_rad), cy + outer * math.cos(edge_rad),
                        ], width=1.0)

            Color(*COVERAGE_COLOR)
            coverage = set()
            for name, bearing in self._bearings.items():
                half = self._fov_half_deg.get(name, 0.0)
                if half > 0:
                    coverage.update((bearing - half, bearing + half))
                else:
                    coverage.add(bearing)
            for bearing in coverage:
                rad = math.radians(bearing)
                Line(points=[cx, cy, cx + r * math.sin(rad), cy + r * math.cos(rad)], width=1.3)

            now = time.time()
            for (sensor_name, kind), trail in self._trails.items():
                bearing = self._bearings.get(sensor_name, 0.0)
                color = MOVING_COLOR if kind == "moving" else STATIONARY_COLOR
                self._draw_trail(trail, color, bearing, cx, cy, r, now)

            Color(*OWNSHIP_COLOR)
            Ellipse(pos=(cx - 3, cy - 3), size=(6, 6))

        for names_key, label in self._bearing_labels.items():
            bearing = self._bearings.get(names_key.split(",")[0], 0.0)
            label.opacity = 1
            rad = math.radians(bearing)
            # Just inside the rim, not outside it: keeps the label inside the
            # scope's own bounds regardless of how little margin a tight
            # layout leaves around the widget (a tag floating past the rim
            # ended up overlapping the HUD row below the scope).
            label_r = max(0.0, r - 18)
            label.center = (cx + label_r * math.sin(rad), cy + label_r * math.cos(rad))

        # 1/2/3/... metre scale along the sector's own start edge -- one of
        # the two straight flat sides of the shape, which for the common
        # single-sensor half-circle IS the base (the bottom, where the flat
        # edge runs). Nudged slightly off the line itself so the numbers
        # don't sit on top of it.
        self._ensure_scale_labels(ring_count)
        edge_rad = math.radians(self._sector_start_deg)
        nudge_rad = math.radians(self._sector_start_deg + 90.0)
        nudge_x, nudge_y = 9.0 * math.sin(nudge_rad), 9.0 * math.cos(nudge_rad)
        for i, label in enumerate(self._scale_labels, start=1):
            if i > ring_count:
                label.opacity = 0
                continue
            label.opacity = 1
            label.text = str(i)
            # Pulled in from the exact rim, same reasoning as the bearing
            # labels above: a label sitting exactly at radius r drifts
            # outside the widget's own bounds once the small perpendicular
            # nudge is added, and ended up overlapping the row below the
            # scope in the real (tightly laid out) app before this margin.
            ring_r = min(r - 12.0, r * i / ring_count)
            label.center = (cx + ring_r * math.sin(edge_rad) + nudge_x,
                             cy + ring_r * math.cos(edge_rad) + nudge_y)

    def _draw_trail(
        self, trail: Deque[TrailPoint], color, bearing_deg: float,
        cx: float, cy: float, r: float, now: float,
    ) -> None:
        n = len(trail)
        for idx, point in enumerate(trail):
            fade = max(0.0, 1.0 - (now - point.t) / TRAIL_FADE_SECONDS)
            if fade <= 0.0:
                continue
            is_latest = idx == n - 1
            if point.bearing_deg is not None:
                x, y = self._point(cx, cy, r, point.bearing_deg, point.distance_cm)
            else:
                x, y = self._point(cx, cy, r, bearing_deg, point.distance_cm)
            if point.moving is not None:
                color = MOVING_COLOR if point.moving else STATIONARY_COLOR
            energy_scale = 0.4 + 0.6 * max(0, min(100, point.energy)) / 100.0
            radius = (7.0 if is_latest else 4.0) * energy_scale * (0.5 + 0.5 * fade)
            Color(*color, fade if not is_latest else min(1.0, fade + 0.3))
            Ellipse(pos=(x - radius, y - radius), size=(radius * 2, radius * 2))
            if is_latest:
                # bright core on top of the glow, so the live position pops
                Color(*color, 1.0)
                core = radius * 0.4
                Ellipse(pos=(x - core, y - core), size=(core * 2, core * 2))
