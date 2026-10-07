# =============================================================================
# telemetry/recorder.py — Team Vajra AeroTHON 2026
#
# Flight data recorder (rulebook: the UAS must record flight data for the jury).
# One flight = three files in logs/ , named by start date and time:
#   flight_YYYYMMDD_HHMMSS.csv           one row per control tick (position, height, battery, ...)
#   flight_YYYYMMDD_HHMMSS_events.txt    every line the mission prints (states, decisions, warnings)
#   flight_YYYYMMDD_HHMMSS_summary.json  result + a copy of every setting in config/params.py
#
# Golden rule: the recorder must NEVER be able to crash or slow down the flight.
# Any disk problem just switches it off (with one printed warning).
# =============================================================================
import csv
import json
import os
import time
from datetime import datetime

import config.params as P

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

COLUMNS = [
    "t_mission_s", "t_wall", "state", "mode", "armed",
    "lat", "lon", "alt_baro_m", "rangefinder_m", "heading_deg",
    "battery_pct", "voltage_v",
    "cmd_vx", "cmd_vy", "cmd_vz", "cmd_yaw_dps",
    "delivery_id", "payload_released",
]


def _try(fn):
    try:
        return fn()
    except Exception:          # noqa: BLE001 - a broken sensor must never stop the log
        return None


def _num(value, digits):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return ""
    return round(float(value), digits)


def _params_snapshot():
    snap = {}
    for name in dir(P):
        if name.isupper():
            value = getattr(P, name)
            try:
                json.dumps(value)
                snap[name] = value
            except (TypeError, ValueError):
                snap[name] = str(value)
    return snap


class CommandTap:
    """Wrap the vehicle so the recorder can log the last velocity command that was sent.
    Everything else passes straight through to the real vehicle."""

    _RESET = {"goto", "rtl", "land", "start_takeoff", "takeoff"}     # the autopilot flies these, not us

    def __init__(self, vehicle):
        self._vehicle = vehicle
        self.last_command = (None, None, None, None)               # vx, vy, vz, yaw_rate_dps

    def __getattr__(self, name):
        attr = getattr(self._vehicle, name)
        if name == "send_body_velocity":
            def tapped(vx, vy, vz, *args, **kwargs):
                self.last_command = (vx, vy, vz, 0.0)
                return attr(vx, vy, vz, *args, **kwargs)
            return tapped
        if name == "send_velocity_yawrate":
            def tapped(vx, vy, vz, yaw_rate_dps, *args, **kwargs):
                self.last_command = (vx, vy, vz, yaw_rate_dps)
                return attr(vx, vy, vz, yaw_rate_dps, *args, **kwargs)
            return tapped
        if name == "hover":
            def tapped(*args, **kwargs):
                self.last_command = (0.0, 0.0, 0.0, 0.0)
                return attr(*args, **kwargs)
            return tapped
        if name in self._RESET:
            def tapped(*args, **kwargs):
                self.last_command = (None, None, None, None)
                return attr(*args, **kwargs)
            return tapped
        return attr


class FlightRecorder:
    def __init__(self, log_dir=None, wall=time.time, mono=time.monotonic, now=datetime.now):
        self.ok = False
        self.base = None
        self.rows = 0
        self._wall, self._mono = wall, mono
        self._csv_f = self._ev_f = self._writer = None
        self._last_sync = mono()
        self._warned = False
        self._started = wall()
        try:
            log_dir = log_dir or P.LOG_DIR
            if not os.path.isabs(log_dir):
                log_dir = os.path.join(_PROJECT_ROOT, log_dir)
            os.makedirs(log_dir, exist_ok=True)
            stamp = now().strftime("%Y%m%d_%H%M%S")
            base, n = os.path.join(log_dir, f"flight_{stamp}"), 1
            while os.path.exists(base + ".csv"):                     # two starts in the same second
                n += 1
                base = os.path.join(log_dir, f"flight_{stamp}_{n}")
            self._csv_f = open(base + ".csv", "w", newline="", encoding="utf-8")
            self._ev_f = open(base + "_events.txt", "w", encoding="utf-8")
            self._writer = csv.writer(self._csv_f)
            self._writer.writerow(COLUMNS)
            self._csv_f.flush()
            self.base = base
            self.ok = True
            self._write_summary("in_progress")                        # settings are saved even if the power dies
        except OSError as exc:
            self._fail(exc)

    # ── public ────────────────────────────────────────────────────────────────
    def record(self, ctx):
        """Write one row. Called by the state machine once per tick."""
        if not self.ok:
            return
        v = ctx.vehicle
        loc = _try(lambda: v.gps_location)
        cmd = _try(lambda: v.last_command)
        if not (isinstance(cmd, tuple) and len(cmd) == 4):
            cmd = (None, None, None, None)
        armed = _try(lambda: v.is_armed)
        row = [
            _num(_try(ctx.mission_elapsed), 2),
            datetime.fromtimestamp(self._wall()).isoformat(timespec="milliseconds"),
            _try(lambda: ctx.state.name) or "",
            _try(lambda: v.mode_name) or "",
            "" if armed is None else int(bool(armed)),
            _num(_try(lambda: loc.lat), 7), _num(_try(lambda: loc.lon), 7),
            _num(_try(lambda: v.altitude), 2), _num(_try(lambda: v.rangefinder_distance), 2),
            _num(_try(lambda: v.heading), 1),
            _num(_try(lambda: v.battery_level), 1), _num(_try(lambda: v.battery_voltage), 2),
            _num(cmd[0], 2), _num(cmd[1], 2), _num(cmd[2], 2), _num(cmd[3], 1),
            _try(lambda: ctx.delivery_id) or "",
            int(bool(_try(lambda: ctx.payload_released))),
        ]
        try:
            self._writer.writerow(row)
            self._csv_f.flush()
            self.rows += 1
            self._maybe_sync()
        except (OSError, ValueError) as exc:
            self._fail(exc)

    def event(self, line):
        """Write one line of the mission's printed log."""
        if not self.ok:
            return
        try:
            self._ev_f.write(str(line) + "\n")
            self._ev_f.flush()
        except (OSError, ValueError) as exc:
            self._fail(exc)

    def close(self, result=None):
        """Finish the files. result = the MissionResult, or None if the mission died unexpectedly."""
        if self.base is None:
            return
        if self.ok:
            self._write_summary("finished" if result is not None else "aborted", result)
        self._shut()
        self.ok = False

    # ── internals ─────────────────────────────────────────────────────────────
    def _maybe_sync(self):
        if self._mono() - self._last_sync >= P.LOG_FSYNC_EVERY_S:
            self._last_sync = self._mono()
            os.fsync(self._csv_f.fileno())
            os.fsync(self._ev_f.fileno())

    def _write_summary(self, status, result=None):
        summary = {
            "status": status,
            "started": datetime.fromtimestamp(self._started).isoformat(timespec="seconds"),
            "rows_recorded": self.rows,
            "final_state": getattr(getattr(result, "final_state", None), "name", None),
            "success": getattr(result, "success", None),
            "reason": getattr(result, "reason", None),
            "payload_released": getattr(result, "payload_released", None),
            "elapsed_s": getattr(result, "elapsed_s", None),
            "history": getattr(result, "history", None),
            "params": _params_snapshot(),
        }
        path = self.base + "_summary.json"
        try:
            with open(path + ".tmp", "w", encoding="utf-8") as f:
                json.dump(summary, f, indent=2, default=str)
            os.replace(path + ".tmp", path)
        except OSError as exc:
            self._fail(exc)

    def _shut(self):
        for f in (self._csv_f, self._ev_f):
            try:
                if f is not None:
                    f.flush()
                    f.close()
            except (OSError, ValueError):
                pass

    def _fail(self, exc):
        if not self._warned:
            print(f"[Recorder] WARNING: flight log switched off ({exc!r}). The flight itself is not affected.")
            self._warned = True
        self.ok = False
        self._shut()