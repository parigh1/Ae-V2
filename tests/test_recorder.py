# tests/test_recorder.py - Phase 7: flight data recorder
import csv
import json
import os
from datetime import datetime
from types import SimpleNamespace

import pytest

from mission.context import MissionContext
from mission.state_machine import StateMachine
from mission.states import State, NEXT, TERMINAL
from telemetry.recorder import COLUMNS, CommandTap, FlightRecorder


# ── fakes ────────────────────────────────────────────────────────────────────
class FakeVehicle:
    def __init__(self):
        self.gps_location = SimpleNamespace(lat=21.1234567, lon=79.7654321, alt=300.0)
        self.altitude, self.rangefinder_distance, self.heading = 5.0, 4.9, 90.0
        self.battery_level, self.battery_voltage = 87.0, 15.8
        self.mode_name, self.is_armed = "GUIDED", True
        self.sent = []

    def send_body_velocity(self, vx, vy, vz):
        self.sent.append(("body", vx, vy, vz))
        return "ok-body"

    def send_velocity_yawrate(self, vx, vy, vz, yaw):
        self.sent.append(("yaw", vx, vy, vz, yaw))

    def hover(self):
        self.sent.append(("hover",))

    def rtl(self):
        self.sent.append(("rtl",))


class Counter:
    """A clock that moves 0.1 s every time it is read, so mission times look real."""
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        self.t += 0.1
        return self.t


def make_recorder(tmp_path, **kw):
    return FlightRecorder(log_dir=str(tmp_path), **kw)


def make_ctx(tmp_path, recorder=None, vehicle=None, safety=None):
    clock = Counter()
    return MissionContext(vehicle=vehicle or FakeVehicle(), safety=safety, clock=clock,
                          sleep=lambda s: None, recorder=recorder)


def read_rows(rec):
    with open(rec.base + ".csv", newline="") as f:
        return list(csv.DictReader(f))


def dummy_handlers():
    """Every state just moves on to the next one (EMERGENCY goes straight to DONE)."""
    h = {s: (lambda ctx, s=s: NEXT[s]) for s in State if s not in TERMINAL and s is not State.EMERGENCY}
    h[State.EMERGENCY] = lambda ctx: State.DONE
    h[State.TAKEOFF] = lambda ctx: (setattr(ctx, "mission_t0", ctx.clock()), NEXT[State.TAKEOFF])[1]
    return h


# ── the files ────────────────────────────────────────────────────────────────
def test_creates_csv_with_header_and_one_row_per_record(tmp_path):
    rec = make_recorder(tmp_path)
    ctx = make_ctx(tmp_path, rec)
    for _ in range(5):
        rec.record(ctx)
    rec.close()
    with open(rec.base + ".csv", newline="") as f:
        lines = list(csv.reader(f))
    assert lines[0] == COLUMNS
    assert len(lines) == 6 and all(len(row) == len(COLUMNS) for row in lines)


def test_row_values_are_the_vehicle_values(tmp_path):
    rec = make_recorder(tmp_path)
    ctx = make_ctx(tmp_path, rec)
    ctx.delivery_id = "B7"
    rec.record(ctx)
    rec.close()
    row = read_rows(rec)[0]
    assert float(row["lat"]) == pytest.approx(21.1234567, abs=1e-7)
    assert float(row["lon"]) == pytest.approx(79.7654321, abs=1e-7)
    assert (row["mode"], row["armed"], row["delivery_id"]) == ("GUIDED", "1", "B7")
    assert float(row["alt_baro_m"]) == 5.0 and float(row["battery_pct"]) == 87.0


def test_file_name_has_date_and_time_and_never_overwrites(tmp_path):
    fixed = lambda: datetime(2026, 10, 7, 14, 30, 5)
    a = make_recorder(tmp_path, now=fixed)
    b = make_recorder(tmp_path, now=fixed)
    assert os.path.basename(a.base) == "flight_20261007_143005"
    assert a.base != b.base and os.path.exists(a.base + ".csv") and os.path.exists(b.base + ".csv")


def test_events_file_gets_every_ctx_log_line(tmp_path):
    rec = make_recorder(tmp_path)
    ctx = make_ctx(tmp_path, rec)
    ctx.log("hello jury")
    ctx.log("second line")
    rec.close()
    text = open(rec.base + "_events.txt", encoding="utf-8").read()
    assert "hello jury" in text and "second line" in text and len(ctx.events) == 2


def test_summary_is_written_at_start_and_holds_the_settings(tmp_path):
    rec = make_recorder(tmp_path)
    first = json.load(open(rec.base + "_summary.json", encoding="utf-8"))
    assert first["status"] == "in_progress"
    assert first["params"]["ALT_DELIVERY"] == 10.0 and first["params"]["SEARCH_STRIP_W"] == 8.0


# ── the recorder must never hurt the flight ──────────────────────────────────
def test_unwritable_folder_switches_the_recorder_off_quietly(tmp_path):
    blocker = tmp_path / "iam_a_file"
    blocker.write_text("x")
    rec = FlightRecorder(log_dir=str(blocker / "logs"))          # a folder cannot be made inside a file
    ctx = make_ctx(tmp_path, rec)
    assert rec.ok is False
    rec.record(ctx); rec.event("x"); rec.close()                 # none of these may raise
    ctx.log("the mission still logs")
    assert ctx.events


def test_broken_sensor_gives_an_empty_cell_not_a_crash(tmp_path):
    class Broken(FakeVehicle):
        @property
        def rangefinder_distance(self):
            raise RuntimeError("sensor died")

        @rangefinder_distance.setter
        def rangefinder_distance(self, v):
            pass

    rec = make_recorder(tmp_path)
    rec.record(make_ctx(tmp_path, rec, vehicle=Broken()))
    rec.close()
    row = read_rows(rec)[0]
    assert row["rangefinder_m"] == "" and row["battery_pct"] == "87.0"


def test_disk_error_while_flying_switches_off_without_raising(tmp_path):
    rec = make_recorder(tmp_path)
    ctx = make_ctx(tmp_path, rec)
    rec.record(ctx)
    rec._csv_f.close()                                           # simulate the SD card disappearing
    rec.record(ctx)
    assert rec.ok is False
    rec.event("still fine")


# ── command tap ──────────────────────────────────────────────────────────────
def test_command_tap_remembers_last_command_and_passes_calls_through(tmp_path):
    v = FakeVehicle()
    tap = CommandTap(v)
    assert tap.send_body_velocity(1.0, 0.5, -0.2) == "ok-body"
    assert tap.last_command == (1.0, 0.5, -0.2, 0.0)
    tap.send_velocity_yawrate(0.4, 0.0, 0.1, 15.0)
    assert tap.last_command == (0.4, 0.0, 0.1, 15.0)
    assert tap.altitude == 5.0 and tap.mode_name == "GUIDED"     # normal attributes pass through
    tap.hover()
    assert tap.last_command == (0.0, 0.0, 0.0, 0.0)
    tap.rtl()
    assert tap.last_command == (None, None, None, None)          # the autopilot is flying now
    assert v.sent[0] == ("body", 1.0, 0.5, -0.2) and v.sent[-1] == ("rtl",)


def test_command_tap_values_reach_the_csv(tmp_path):
    rec = make_recorder(tmp_path)
    tap = CommandTap(FakeVehicle())
    ctx = make_ctx(tmp_path, rec, vehicle=tap)
    tap.send_velocity_yawrate(1.2, -0.3, 0.0, 30.0)
    rec.record(ctx)
    rec.close()
    row = read_rows(rec)[0]
    assert (row["cmd_vx"], row["cmd_vy"], row["cmd_yaw_dps"]) == ("1.2", "-0.3", "30.0")


# ── inside the state machine ─────────────────────────────────────────────────
def test_full_dummy_mission_records_rows_events_and_summary(tmp_path):
    rec = make_recorder(tmp_path)
    ctx = make_ctx(tmp_path, rec)
    result = StateMachine(ctx, dummy_handlers()).run()
    assert result.final_state is State.DONE and result.success
    rows = read_rows(rec)
    assert len(rows) >= len(State) - 2
    assert {r["state"] for r in rows} >= {"INIT", "TAKEOFF", "LAND"}
    events = open(rec.base + "_events.txt", encoding="utf-8").read()
    assert "→ TAKEOFF" in events and "→ DONE" in events
    summary = json.load(open(rec.base + "_summary.json", encoding="utf-8"))
    assert summary["status"] == "finished" and summary["final_state"] == "DONE"
    assert summary["success"] is True and summary["rows_recorded"] == len(rows)
    assert rec.ok is False                                       # closed


def test_pilot_override_still_closes_the_log(tmp_path):
    rec = make_recorder(tmp_path)
    stop = SimpleNamespace(check=lambda: SimpleNamespace(action="stop", reason="pilot_override (mode STABILIZE)"))
    ctx = make_ctx(tmp_path, rec, safety=stop)
    result = StateMachine(ctx, dummy_handlers()).run()
    assert result.success is False
    summary = json.load(open(rec.base + "_summary.json", encoding="utf-8"))
    assert summary["status"] == "finished" and "pilot_override" in summary["reason"]


def test_hard_abort_still_closes_the_log_and_marks_it_aborted(tmp_path):
    rec = make_recorder(tmp_path)
    ctx = make_ctx(tmp_path, rec)
    handlers = dummy_handlers()
    handlers[State.SCAN_START_QR] = lambda c: (_ for _ in ()).throw(KeyboardInterrupt())   # Ctrl+C
    with pytest.raises(KeyboardInterrupt):
        StateMachine(ctx, handlers).run()
    summary = json.load(open(rec.base + "_summary.json", encoding="utf-8"))
    assert summary["status"] == "aborted"
    assert len(read_rows(rec)) > 3                               # what was flown so far is on disk


def test_mission_without_a_recorder_is_unchanged(tmp_path):
    ctx = make_ctx(tmp_path, None)
    result = StateMachine(ctx, dummy_handlers()).run()
    assert result.final_state is State.DONE and ctx.recorder is None