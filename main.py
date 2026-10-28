# =============================================================================
# main.py — Team Vajra AeroTHON 2026
#
# Entry point for the autonomous mission. Run on the Raspberry Pi.
#
# Usage:
#   python main.py                          # real hardware, default serial port
#   python main.py --connect /dev/ttyAMA0 --baud 921600   # specify the port
#   python main.py --connect udp:127.0.0.1:14550          # SITL over UDP
#   python main.py --sim                    # no Pixhawk at all (MockVehicle + FakeCamera)
#   python main.py --log-dir /data/logs     # write flight logs elsewhere
#
# The script blocks on a "GO" confirmation before arming. Press Enter to fly
# or Ctrl+C to abort. Ctrl+C during the mission triggers an RTL.
# =============================================================================

import argparse
import sys
import time
import traceback
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
from config.params import (
    VEHICLE_CONNECT_STR, VEHICLE_BAUD,
    CAM_IMG_W, CAM_IMG_H,
    LOG_DIR,
)
from hardware.camera import build_camera_rig, FakeCamera
from hardware.vehicle import MockVehicle
from mission.context import MissionContext
from mission.handlers import default_handlers
from mission.state_machine import StateMachine
from navigation.altitude import AltitudeController
from navigation.visual_servo import VisualServo
from payload.release import PayloadReleaser
from safety.watchdog import Watchdog
from telemetry.recorder import CommandTap, FlightRecorder
from vision.qr_system import QRSystem


# ── argument parsing ──────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Team Vajra AeroTHON 2026 — autonomous mission")
    p.add_argument("--connect", default=VEHICLE_CONNECT_STR,
                   help=f"DroneKit connection string (default: {VEHICLE_CONNECT_STR})")
    p.add_argument("--baud", type=int, default=VEHICLE_BAUD,
                   help=f"Serial baud rate (default: {VEHICLE_BAUD})")
    p.add_argument("--sim", action="store_true",
                   help="Use MockVehicle + FakeCamera instead of real hardware")
    p.add_argument("--log-dir", default=LOG_DIR,
                   help=f"Folder to write flight logs (default: {LOG_DIR})")
    p.add_argument("--no-wait", action="store_true",
                   help="Skip the GO/abort confirmation (useful in automated tests)")
    return p.parse_args()


# ── vehicle connection ────────────────────────────────────────────────────────
def connect_vehicle(args):
    if args.sim:
        print("[main] SIM mode: using MockVehicle (no Pixhawk needed)")
        return MockVehicle(verbose=True)

    print(f"[main] Connecting to Pixhawk on '{args.connect}' at {args.baud} baud …")
    try:
        import dronekit
        vehicle = dronekit.connect(args.connect, baud=args.baud, wait_ready=True,
                                   heartbeat_timeout=30)
        print("[main] Connected.")
        from hardware.vehicle import Vehicle
        return Vehicle(vehicle)
    except Exception as exc:
        print(f"[main] ERROR: could not connect to the Pixhawk: {exc!r}")
        print("       Is the cable plugged in? Is the baud rate correct?")
        print("       Use --sim to run without hardware.")
        sys.exit(1)


# ── camera ────────────────────────────────────────────────────────────────────
def build_camera(args):
    if args.sim:
        import numpy as np
        blank = np.zeros((CAM_IMG_H, CAM_IMG_W, 3), dtype="uint8")
        return build_camera_rig(mode="single_fixed",
                                cameras={"main": FakeCamera(blank)})
    # Real Pi camera: auto-detected by build_camera_rig via CAMERA_BACKEND in params
    return build_camera_rig()


# ── operator confirmation ─────────────────────────────────────────────────────
def wait_for_go(args):
    if args.no_wait:
        return True
    print()
    print("=" * 60)
    print("  Team Vajra AeroTHON 2026 — autonomous mission")
    print("=" * 60)
    print()
    print("  Pre-flight checklist:")
    print("    [ ] Props on and tight")
    print("    [ ] Battery plugged in and level confirmed in GCS")
    print("    [ ] RC transmitter on, mode switch in GUIDED")
    print("    [ ] Area clear of people")
    print("    [ ] Flight controller GPS lock (solid green LED)")
    print()
    try:
        answer = input("  Type GO and press Enter to arm and fly, or Ctrl+C to abort: ")
    except (EOFError, KeyboardInterrupt):
        print("\n[main] Aborted by operator.")
        return False
    if answer.strip().upper() != "GO":
        print("[main] Aborted (you did not type GO).")
        return False
    print()
    return True


# ── main ──────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()

    # 1. connect to hardware
    raw_vehicle = connect_vehicle(args)

    # 2. wrap the vehicle so the recorder can log every command
    vehicle = CommandTap(raw_vehicle)

    # 3. flight recorder (started before preflight so the whole run is on disk)
    recorder = FlightRecorder(log_dir=args.log_dir)
    if recorder.ok:
        print(f"[main] Flight log: {recorder.base}")
    else:
        print("[main] WARNING: flight log could not be opened (check the logs/ folder).")

    # 4. operator confirmation before arming
    if not wait_for_go(args):
        recorder.close()
        if not args.sim:
            raw_vehicle.close()
        return

    # 5. build all the mission parts
    rig = build_camera(args)
    qr_cam = rig.view("down")

    altitude  = AltitudeController(vehicle)
    servo     = VisualServo(vehicle)
    watchdog  = Watchdog(vehicle)
    payload   = PayloadReleaser(vehicle)
    qr_system = QRSystem(qr_cam, vehicle)

    ctx = MissionContext(
        vehicle=vehicle,
        rig=rig,
        qr=qr_system,
        altitude=altitude,
        servo=servo,
        safety=watchdog,
        payload=payload,
        recorder=recorder,
    )

    # 6. fly
    result = None
    try:
        result = StateMachine(ctx, default_handlers()).run()
    except KeyboardInterrupt:
        print("\n[main] Ctrl+C — commanding RTL …")
        try:
            raw_vehicle.rtl()
        except Exception:
            pass
    except Exception:
        print("[main] Unexpected error:")
        traceback.print_exc()
        print("[main] Commanding RTL …")
        try:
            raw_vehicle.rtl()
        except Exception:
            pass
    finally:
        # recorder.close() is also called by StateMachine.run() in its finally block,
        # but close() is idempotent so calling it twice is harmless
        recorder.close(result)
        if not args.sim:
            try:
                raw_vehicle.close()
            except Exception:
                pass

    # 7. print a summary
    print()
    if result is None:
        print("[main] Mission did not complete normally.")
    else:
        icon = "✓" if result.success else "✗"
        print(f"[main] {icon} {result.reason}")
        print(f"       Elapsed: {result.elapsed_s:.1f} s")
        print(f"       Payload released: {result.payload_released}")
        print(f"       States flown: {' → '.join(s for s, _ in result.history)}")
    if recorder.base:
        print(f"       Log: {recorder.base}.csv")
    print()


if __name__ == "__main__":
    main()