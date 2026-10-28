"""
test_connect.py — verify the Pi can talk to the Pixhawk and all preflight checks pass.
No arming. No motors. Safe to run with props on or off.

Usage (Pi, props off, FC powered):
    python tests/scripts/test_connect.py --connect /dev/ttyAMA0 --baud 921600

Usage (laptop, SITL running in Mission Planner):
    python tests/scripts/test_connect.py --connect udp:127.0.0.1:14550
"""
import argparse, sys, time
sys.path.insert(0, '.')
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--connect', default='udp:127.0.0.1:14550')
    ap.add_argument('--baud', type=int, default=921600)
    args = ap.parse_args()

    print(f"Connecting to {args.connect} …")
    import dronekit
    v = dronekit.connect(args.connect, baud=args.baud, wait_ready=True, heartbeat_timeout=30)
    print("Connected.")
    print()

    # Read every property the code uses
    loc = v.location.global_relative_frame
    print(f"  Mode:        {v.mode.name}")
    print(f"  Armed:       {v.armed}")
    print(f"  GPS fix:     {v.gps_0.fix_type}  (need 3+)")
    print(f"  Satellites:  {v.gps_0.satellites_visible}  (need 8+)")
    print(f"  EKF ok:      {v.ekf_ok}")
    print(f"  Battery:     {v.battery.level}%  {v.battery.voltage:.2f}V")
    print(f"  Altitude:    {v.location.global_relative_frame.alt:.2f} m")
    print(f"  Heading:     {v.heading} deg")
    print(f"  Lat/Lon:     {loc.lat:.7f}, {loc.lon:.7f}")
    print(f"  Last hb age: {time.monotonic() - v.last_heartbeat:.2f} s")
    print()

    # Check every preflight condition
    from config.params import (PREFLIGHT_MIN_BATTERY_PCT, PREFLIGHT_MIN_GPS_FIX,
                                PREFLIGHT_MIN_SATS, REQUIRED_PARAMS)
    problems = []
    if v.battery.level is None:
        problems.append("battery level not reported")
    elif v.battery.level < PREFLIGHT_MIN_BATTERY_PCT:
        problems.append(f"battery {v.battery.level}% < {PREFLIGHT_MIN_BATTERY_PCT}%")
    if v.gps_0.fix_type < PREFLIGHT_MIN_GPS_FIX:
        problems.append(f"GPS fix type {v.gps_0.fix_type} < {PREFLIGHT_MIN_GPS_FIX}")
    if v.gps_0.satellites_visible < PREFLIGHT_MIN_SATS:
        problems.append(f"only {v.gps_0.satellites_visible} sats < {PREFLIGHT_MIN_SATS}")
    if not v.ekf_ok:
        problems.append("EKF not ok")

    print("Checking REQUIRED_PARAMS …")
    for name, expected in REQUIRED_PARAMS.items():
        val = v.parameters.get(name)
        status = "✓" if val == expected else f"✗ (got {val}, need {expected})"
        print(f"  {name}: {status}")
        if val != expected:
            problems.append(f"{name} = {val}, need {expected}")

    print()
    if problems:
        print("PREFLIGHT WOULD FAIL:")
        for p in problems:
            print(f"  ✗ {p}")
    else:
        print("✓ All preflight checks pass. Ready to arm (but this script does NOT arm).")

    v.close()

if __name__ == '__main__':
    main()