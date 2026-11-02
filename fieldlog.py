"""Rotating file log of what the sensor actually reported.

The screen only shows the current moment; this keeps a record. It logs state
*transitions* (room goes from clear to occupied, or the reverse) rather than
every frame -- at 15 Hz that would be thousands of lines a minute for no
benefit -- plus a periodic heartbeat while a target is being tracked, and
every connection status change (link up, link lost, error, reconnecting).

    python main.py --mock --log           # writes into ./logs/
    python main.py --mock --log /path     # writes into an explicit directory
"""

from __future__ import annotations

import logging
import logging.handlers
import time
from pathlib import Path
from typing import Optional

import ld2410
from ld2410 import TargetFrame

# How often to re-log a snapshot while a target is continuously present, so a
# long dwell still leaves a trail instead of one line and silence.
HEARTBEAT_INTERVAL = 5.0

LOG_FORMAT = "%(asctime)s.%(msecs)03d  %(message)s"
DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


def default_log_dir() -> Path:
    """Where to put logs when the caller doesn't specify: app-private storage
    on Android, a ./logs folder next to the script everywhere else."""
    try:
        from android.storage import app_storage_path  # type: ignore

        return Path(app_storage_path()) / "logs"
    except Exception:
        pass
    return Path(__file__).resolve().parent / "logs"


def _build_logger(log_dir: Path, max_bytes: int, backups: int) -> logging.Logger:
    log_dir.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("ld2410.field")
    logger.setLevel(logging.INFO)
    logger.propagate = False
    # Reconfiguring (e.g. a second FieldLogger in the same process, as in
    # tests) should not stack duplicate handlers writing the same line twice.
    for handler in list(logger.handlers):
        logger.removeHandler(handler)
        handler.close()

    handler = logging.handlers.RotatingFileHandler(
        log_dir / "ld2410_field.log",
        maxBytes=max_bytes,
        backupCount=backups,
        encoding="utf-8",
    )
    handler.setFormatter(logging.Formatter(LOG_FORMAT, datefmt=DATE_FORMAT))
    logger.addHandler(handler)
    return logger


class FieldLogger:
    """Hook this onto LD2410Reader's on_frame/on_status callbacks.

    Safe to call from the reader thread: it only does file I/O, which the
    reader already tolerates (its callback wrapper swallows exceptions), and
    it only writes on transitions/heartbeats, not every frame, so it never
    adds meaningful latency to the read loop.
    """

    def __init__(
        self,
        log_dir: Optional[Path] = None,
        max_bytes: int = 2_000_000,
        backups: int = 5,
        heartbeat_interval: float = HEARTBEAT_INTERVAL,
    ):
        self.log_dir = Path(log_dir) if log_dir else default_log_dir()
        self._logger = _build_logger(self.log_dir, max_bytes, backups)
        self._heartbeat_interval = heartbeat_interval
        # Keyed by sensor name ("" for the default single-sensor caller), so
        # a multi-sensor setup tracks each one's dwell independently.
        self._last_state: dict = {}
        self._last_heartbeat: dict = {}
        self.path = self.log_dir / "ld2410_field.log"
        self._logger.info("=== field logger started ===")

    def on_status(self, state: str, detail: str, sensor: str = "") -> None:
        tag = "[%s] " % sensor if sensor else ""
        self._logger.info("%sLINK %s  %s", tag, state.upper(), detail)
        if state in (ld2410.STATUS_CONNECTING, ld2410.STATUS_ERROR, ld2410.STATUS_STOPPED):
            # A reconnect or restart invalidates any in-progress dwell; the
            # next frame should be logged fresh rather than treated as a
            # continuation of whatever was being tracked before the drop.
            self._last_state.pop(sensor, None)
            self._last_heartbeat.pop(sensor, None)

    def on_frame(self, frame: TargetFrame, sensor: str = "") -> None:
        now = frame.timestamp or time.time()
        tag = "[%s] " % sensor if sensor else ""

        if hasattr(frame, "active_targets"):
            self._on_trajectory_frame(frame, tag, sensor, now)
            return

        if frame.state != self._last_state.get(sensor):
            if frame.has_target:
                self._logger.info(
                    "%sTARGET %s  %d cm  energy %d",
                    tag, frame.state_name, frame.distance_cm, frame.energy,
                )
            else:
                self._logger.info("%sTARGET CLEAR", tag)
            self._last_state[sensor] = frame.state
            self._last_heartbeat[sensor] = now
            return

        last_heartbeat = self._last_heartbeat.get(sensor, 0.0)
        if frame.has_target and now - last_heartbeat >= self._heartbeat_interval:
            self._logger.info(
                "%s  ...  %s  %d cm  energy %d  (still present)",
                tag, frame.state_name, frame.distance_cm, frame.energy,
            )
            self._last_heartbeat[sensor] = now

    def _on_trajectory_frame(self, frame, tag: str, sensor: str, now: float) -> None:
        """LD2450 frames: the "state" worth logging a transition on is how
        many targets are in view; positions go in the line itself."""
        count = len(frame.active_targets)
        if count != self._last_state.get(sensor):
            if count:
                self._logger.info("%sTARGETS %d  %s", tag, count, frame)
            else:
                self._logger.info("%sTARGET CLEAR", tag)
            self._last_state[sensor] = count
            self._last_heartbeat[sensor] = now
            return

        if count and now - self._last_heartbeat.get(sensor, 0.0) >= self._heartbeat_interval:
            self._logger.info("%s  ...  %s  (still present)", tag, frame)
            self._last_heartbeat[sensor] = now

    def close(self) -> None:
        self._logger.info("=== field logger stopped ===")
        for handler in list(self._logger.handlers):
            self._logger.removeHandler(handler)
            handler.close()


def _main() -> None:
    """Tail-style smoke test: python fieldlog.py --mock [--seconds 20]"""
    import argparse

    from ld2410 import LD2410Reader
    from serial_backend import make_link_factory

    ap = argparse.ArgumentParser(description="Run the field logger standalone")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=256000)
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--dir", help="log directory (default: ./logs)")
    args = ap.parse_args()

    field_log = FieldLogger(log_dir=args.dir)
    print("writing to", field_log.path)
    reader = LD2410Reader(
        open_link=make_link_factory(port=args.port, baud=args.baud, mock=args.mock),
        on_frame=field_log.on_frame,
        on_status=field_log.on_status,
    )
    reader.start()
    time.sleep(args.seconds)
    reader.stop()
    field_log.close()
    print(Path(field_log.path).read_text(encoding="utf-8"))


if __name__ == "__main__":
    _main()
