"""Multi-sensor configuration.

The LD2410C reports distance only -- no bearing. The only way the app can
show a real direction is if *you* tell it where each physical sensor is
aimed, by naming it and giving it a fixed mounting bearing. The scope then
plots that sensor's target at that real angle. Nothing here infers or
guesses an angle; it only draws what you configured.

    --sensors "FRONT:0,REAR:180"              # auto-detect ports by order
    --sensors "FRONT:0:COM5,REAR:180:COM7"     # pin each sensor to a port

Bearing is degrees, 0 = straight ahead ("forward"), increasing clockwise
(90 = right, 180 = behind, 270 = left) -- a compass bearing relative to
however you've mounted the sensor(s), not true/magnetic north.

With no --sensors flag at all, the app runs the original single-sensor
("SENSOR", bearing 0) setup driven by --port/--mock, unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional


@dataclass(frozen=True)
class SensorConfig:
    name: str
    bearing_deg: float
    port: Optional[str] = None  # None = auto-detect by position in the list


def parse_sensor_spec(spec: str) -> List[SensorConfig]:
    """Parse "NAME:BEARING[:PORT],NAME:BEARING[:PORT],...".

    Raises ValueError with a message meant to be shown to the user as-is.
    """
    sensors: List[SensorConfig] = []
    seen = set()
    for raw in spec.split(","):
        raw = raw.strip()
        if not raw:
            continue
        parts = raw.split(":")
        if len(parts) not in (2, 3):
            raise ValueError(
                'bad --sensors entry "%s" (want NAME:BEARING or NAME:BEARING:PORT)' % raw
            )
        name = parts[0].strip()
        if not name:
            raise ValueError('sensor name cannot be empty in "%s"' % raw)
        if name in seen:
            raise ValueError('duplicate sensor name "%s" in --sensors' % name)
        seen.add(name)
        try:
            bearing = float(parts[1])
        except ValueError:
            raise ValueError(
                'bad bearing "%s" for sensor "%s" (degrees, 0 = forward)' % (parts[1], name)
            )
        port = parts[2].strip() if len(parts) == 3 and parts[2].strip() else None
        sensors.append(SensorConfig(name=name, bearing_deg=bearing % 360.0, port=port))

    if not sensors:
        raise ValueError("--sensors given but no sensors parsed out of it")
    return sensors
