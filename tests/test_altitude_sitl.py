#!/usr/bin/env python3
"""
tests/test_altitude_sitl.py
============================
Tests the AltitudeController against SITL through all 4 mission setpoints:
    3m (corridor) → 5m (QR scan) → 10m (delivery search) → 5m (drop) → land

Run with SITL already running:
    python tests/test_altitude_sitl.py

Watch the map window — you should see the drone climb/descend cleanly
through each altitude with no overshoot or oscillation.
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from config.params import SITL_CONNECTION, SITL_BAUD
from hardware.vehicle import Vehicle
from navigation.altitude import AltitudeController


MISSION_ALTITUDES = [
    (3.0,  "Corridor altitude"),
    (5.0,  "Start QR scan altitude"),
    (10.0, "Delivery zone altitude"),
    (5.0,  "Payload drop altitude"),
]


def main():
    print("=" * 60)
    print("  Altitude PID Test — SITL")
    print("=" * 60)

    vehicle = None
    try:
        vehicle = Vehicle(SITL_CONNECTION, baud=SITL_BAUD)
        alt_ctrl = AltitudeController(vehicle)

        # Arm and takeoff to first target
        vehicle.arm()
        vehicle.takeoff(3.0)

        results = []
        for target, label in MISSION_ALTITUDES:
            print(f"\n── {label}: targeting {target}m ──")
            t_start = time.time()
            ok = alt_ctrl.hold_until_reached(target, timeout=25.0)
            elapsed = time.time() - t_start

            # Hold at altitude for 5 seconds to confirm stability
            if ok:
                print(f"  Holding {target}m for 5s stability check...")
                alts = []
                for _ in range(10):
                    alts.append(alt_ctrl.get_altitude())
                    alt_ctrl.update()
                    time.sleep(0.5)
                variance = max(alts) - min(alts)
                print(f"  Altitude variance over 5s: {variance:.3f}m  "
                      f"(good if < 0.3m)")
                results.append((label, target, ok, elapsed, variance))
            else:
                results.append((label, target, False, elapsed, 999))

        # Land
        vehicle.land()
        time.sleep(5)

        # Print summary
        print("\n" + "=" * 60)
        print("  RESULTS")
        print("=" * 60)
        all_pass = True
        for label, target, ok, elapsed, var in results:
            status = "✓ PASS" if ok else "✗ FAIL"
            if not ok:
                all_pass = False
            print(f"  {status}  {label} ({target}m)  "
                  f"time={elapsed:.1f}s  variance={var:.3f}m")

        print("\n[Test]", "✓ ALL PASS" if all_pass else "✗ SOME FAILURES")
        if not all_pass:
            print("[Hint] If altitude doesn't stabilize: check ALT_KP/KI/KD in params.py")
            print("       If it oscillates: reduce ALT_KP or increase ALT_KD")
            print("       If it's too slow: increase ALT_KP")

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
