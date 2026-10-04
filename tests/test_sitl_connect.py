#!/usr/bin/env python3
"""
tests/test_sitl_connect.py
==========================
FIRST test to run after SITL is up.
Verifies: connection → arm → takeoff → altitude hold → land

Run with SITL already running:
    sim_vehicle.py -v ArduCopter --console --map

Then in a second terminal:
    cd ~/drone_project/Ae-V2
    source ../venv/bin/activate
    python tests/test_sitl_connect.py

Expected output:
    [Vehicle] Connecting to: tcp:127.0.0.1:5760
    [Vehicle] Connected. Firmware: ...
    [Vehicle] Mode: GUIDED
    [Vehicle] Armed ✓
    [Vehicle] Taking off to 5.0m
    [Vehicle] Altitude: 0.xx → ... → 4.6xm  (climbs)
    [Vehicle] Takeoff complete at x.xxm
    [HOLD] Holding 5m for 10s ...  altitude: x.xxm  battery: xxx%
    [Vehicle] Mode: LAND
    [Test] PASS
"""

import sys
import os
import time

# Make sure project root is on the path when running from any directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from config.params import SITL_CONNECTION, SITL_BAUD, ALT_START_QR
from hardware.vehicle import Vehicle


def main():
    print("=" * 60)
    print("  SITL Connection Test — Team Vajra AeroTHON 2026")
    print("=" * 60)

    vehicle = None
    try:
        # ── Step 1: Connect ────────────────────────────────────────────
        vehicle = Vehicle(SITL_CONNECTION, baud=SITL_BAUD)

        # ── Step 2: Pre-arm checks ─────────────────────────────────────
        print(f"\n[Pre-arm] Battery:    {vehicle.battery_level:.0f}%  "
              f"({vehicle.battery_voltage:.2f}V)")
        print(f"[Pre-arm] GPS:        {vehicle.gps_location.lat:.6f}, "
              f"{vehicle.gps_location.lon:.6f}")
        print(f"[Pre-arm] Altitude:   {vehicle.altitude:.2f}m")
        print(f"[Pre-arm] Heading:    {vehicle.heading:.1f}°")

        # ── Step 3: Arm in GUIDED mode ─────────────────────────────────
        vehicle.arm()

        # ── Step 4: Takeoff to 5m ──────────────────────────────────────
        vehicle.takeoff(ALT_START_QR)

        # ── Step 5: Hold altitude for 10 seconds ──────────────────────
        print(f"\n[HOLD] Holding {ALT_START_QR}m for 10 seconds...")
        t_start = time.time()
        while time.time() - t_start < 10:
            alt = vehicle.altitude
            batt = vehicle.battery_level
            print(f"[HOLD]   altitude: {alt:.2f}m   battery: {batt:.0f}%")
            time.sleep(1)

        # ── Step 6: Test velocity commands ─────────────────────────────
        print("\n[VEL] Testing NED velocity — move North 1s")
        vehicle.send_ned_velocity(0.5, 0, 0, duration=1.0)
        print("[VEL] Stop")
        vehicle.hover()
        time.sleep(1)

        print("[VEL] Testing NED velocity — move East 1s")
        vehicle.send_ned_velocity(0, 0.5, 0, duration=1.0)
        vehicle.hover()
        time.sleep(1)

        # ── Step 7: Land ───────────────────────────────────────────────
        print("\n[LAND] Landing...")
        vehicle.land()
        time.sleep(5)

        print("\n[Test] ✓  PASS — SITL connection, arm, takeoff, hold, land all OK")
        print("[Test] You are ready to proceed to the next module.")

    except Exception as e:
        print(f"\n[Test] ✗  FAIL — {e}")
        import traceback
        traceback.print_exc()
        if vehicle:
            try:
                vehicle.rtl()
            except Exception:
                pass
        sys.exit(1)

    finally:
        if vehicle:
            vehicle.close()


if __name__ == "__main__":
    main()
