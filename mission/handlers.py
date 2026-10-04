# =============================================================================
# mission/handlers.py — Team Vajra AeroTHON 2026
#
# One small function per state. The state machine calls handler(ctx) every tick:
#   return None            -> stay in this state
#   return State.SOMETHING -> go there
# ctx.sd is scratch memory for the CURRENT state (wiped on entry).
#
# Real now:  INIT .. SCAN_START_QR, altitude changes, SEARCH (hover & look),
#            CENTER_OVER_QR, DESCEND_TO_DROP, DEPLOY_PAYLOAD, RETURN_TO_HOME,
#            LAND, EMERGENCY.
# STUBS (marked) wait for their own phase: banner finding, corridor flying.
# =============================================================================
from typing import Dict, Optional

from config.params import (
    ALT_START_QR, ALT_CORRIDOR, ALT_DELIVERY, ALT_PAYLOAD_DROP,
    START_QR_FORWARD_M, START_QR_MOVE_SPEED,
    PREFLIGHT_MIN_BATTERY_PCT, REQUIRED_PARAMS,
    SERVO_CONFIRM_FRAMES, SERVO_LOST_FRAMES_MAX, CENTER_RETRY_MAX, HOME_RADIUS_M,
)
from mission.states import State, NEXT
from vision.qr_scanner import QRConfirmer


def _hold_altitude(ctx):
    """Send one 'stay at the target height, don't drift' command."""
    ctx.vehicle.send_body_velocity(0.0, 0.0, ctx.altitude.compute())


# ── start-up ─────────────────────────────────────────────────────────────────
def h_init(ctx):
    ctx.log("INIT: systems up")
    return State.PREFLIGHT


def h_preflight(ctx):
    v = ctx.vehicle
    if v.battery_level < PREFLIGHT_MIN_BATTERY_PCT:
        ctx.data["emergency_reason"] = f"preflight: battery {v.battery_level:.0f}% < {PREFLIGHT_MIN_BATTERY_PCT}%"
        return State.EMERGENCY
    if not v.verify_params(REQUIRED_PARAMS):
        ctx.data["emergency_reason"] = "preflight: failsafe parameters wrong"
        return State.EMERGENCY
    if ctx.payload is not None:
        ctx.payload.hold()
    ctx.log("PREFLIGHT: all checks passed")
    return State.ARM


def h_arm(ctx):
    ctx.vehicle.arm()
    return State.TAKEOFF


def h_takeoff(ctx):
    sd, v = ctx.sd, ctx.vehicle
    if not sd.get("started"):
        ctx.mission_t0 = ctx.clock()                     # rulebook: clock starts at take-off throttle
        loc = v.gps_location
        ctx.home = (loc.lat, loc.lon, loc.alt)
        v.start_takeoff(ALT_START_QR)
        sd["started"] = True
        return None
    if v.altitude >= 0.92 * ALT_START_QR:
        ctx.altitude.set_target(ALT_START_QR)
        return State.MOVE_TO_QR_POINT


def h_move_to_qr_point(ctx):
    hop_time = START_QR_FORWARD_M / START_QR_MOVE_SPEED
    if ctx.state_age() < hop_time:
        ctx.vehicle.send_body_velocity(START_QR_MOVE_SPEED, 0.0, ctx.altitude.compute())
        return None
    ctx.vehicle.hover()
    return State.SCAN_START_QR


def h_scan_start_qr(ctx):
    if "confirmer" not in ctx.sd:
        ctx.sd["confirmer"] = QRConfirmer()
    _hold_altitude(ctx)
    found = ctx.qr.scan_step(ctx.sd["confirmer"])
    if found:
        ctx.delivery_id = found
        ctx.log(f"Start QR confirmed: '{found}'")
        return State.FIND_BANNER_FWD
    return None    # timeout (state clock) -> FIND_BANNER_FWD with delivery_id None


# ── helpers for "go to this height" states ───────────────────────────────────
def altitude_state(state: State, target_m: float, stay_over_target: bool = False):
    def handler(ctx):
        sd = ctx.sd
        if not sd.get("started"):
            ctx.altitude.set_target(target_m)
            ctx.servo.reset()
            sd["started"] = True
        vx = vy = 0.0
        if stay_over_target:                              # keep the drop target under us
            fix = ctx.qr.find_target()
            if fix is not None:
                vx, vy = ctx.servo.compute(fix.forward_m, fix.right_m, ctx.clock())
        ctx.vehicle.send_body_velocity(vx, vy, ctx.altitude.compute())
        if ctx.altitude.reached_target():
            return NEXT[state]
    return handler


# ── STUBS (their own phases) ─────────────────────────────────────────────────
def stub(state: State):
    def handler(ctx):
        _hold_altitude(ctx)
        if not ctx.sd.get("said"):
            ctx.log(f"   (STUB) {state.name}: not built yet, skipping")
            ctx.sd["said"] = True
        return NEXT[state]
    return handler


# ── delivery zone ────────────────────────────────────────────────────────────
def h_search_delivery(ctx):
    """Hover and look until the matching QR shows up.
    (Lawnmower pattern + red-zone avoidance arrive in the search phase.)"""
    _hold_altitude(ctx)
    if ctx.delivery_id is None:
        ctx.log("   no delivery ID -> skipping the drop, heading home")
        return State.FIND_BANNER_RTN
    if ctx.qr.find_target() is not None:
        ctx.log("   target QR spotted")
        return State.CENTER_OVER_QR
    return None    # state clock -> FIND_BANNER_RTN


def h_center_over_qr(ctx):
    sd = ctx.sd
    if not sd.get("started"):
        ctx.servo.reset()
        ctx.altitude.set_target(ALT_DELIVERY)
        sd.update(started=True, centered=0, lost=0)
    vz = ctx.altitude.compute()
    fix = ctx.qr.find_target()

    if fix is None:
        sd["lost"] += 1
        sd["centered"] = 0
        ctx.servo.reset()
        ctx.vehicle.send_body_velocity(0.0, 0.0, vz)
        if sd["lost"] > SERVO_LOST_FRAMES_MAX:
            n = ctx.data.get("center_retries", 0) + 1
            ctx.data["center_retries"] = n
            if n <= CENTER_RETRY_MAX:
                ctx.log(f"   target lost -> searching again ({n}/{CENTER_RETRY_MAX})")
                return State.SEARCH_DELIVERY
            ctx.log("   target lost too often -> giving up on the drop")
            return State.FIND_BANNER_RTN
        return None

    sd["lost"] = 0
    if ctx.servo.is_centered(fix.forward_m, fix.right_m):
        sd["centered"] += 1
        ctx.vehicle.send_body_velocity(0.0, 0.0, vz)
        if sd["centered"] >= SERVO_CONFIRM_FRAMES:
            ctx.log(f"   centred (fwd {fix.forward_m:+.2f} m, right {fix.right_m:+.2f} m)")
            return State.DESCEND_TO_DROP
    else:
        sd["centered"] = 0
        vx, vy = ctx.servo.compute(fix.forward_m, fix.right_m, ctx.clock())
        ctx.vehicle.send_body_velocity(vx, vy, vz)
    return None


def h_deploy_payload(ctx):
    _hold_altitude(ctx)
    if ctx.payload is None:
        ctx.log("   (no payload mechanism attached)")
        return State.CLIMB_AFTER_DROP
    if ctx.payload.step(ctx.clock()):
        ctx.payload_released = True
        ctx.log("   payload released")
        return State.CLIMB_AFTER_DROP


# ── going home ───────────────────────────────────────────────────────────────
def h_return_to_home(ctx):
    lat, lon, _ = ctx.home
    if not ctx.sd.get("started"):
        ctx.vehicle.goto(lat, lon, max(ctx.vehicle.altitude, ALT_CORRIDOR))
        ctx.sd["started"] = True
        return None
    if ctx.vehicle.distance_to_m(lat, lon) < HOME_RADIUS_M:
        return State.LAND


def h_land(ctx):
    if not ctx.sd.get("started"):
        ctx.vehicle.land()
        ctx.sd["started"] = True
        return None
    if not ctx.vehicle.is_armed:                         # ArduPilot disarms after touchdown
        return State.DONE


def h_emergency(ctx):
    v = ctx.vehicle
    if not ctx.sd.get("started"):
        ctx.sd["started"] = True
        if not v.is_armed:                               # never left the ground
            return State.DONE
        v.rtl()
        return None
    if not v.is_armed:
        return State.DONE


def default_handlers() -> Dict[State, callable]:
    S = State
    return {
        S.INIT: h_init,
        S.PREFLIGHT: h_preflight,
        S.ARM: h_arm,
        S.TAKEOFF: h_takeoff,
        S.MOVE_TO_QR_POINT: h_move_to_qr_point,
        S.SCAN_START_QR: h_scan_start_qr,
        S.FIND_BANNER_FWD: stub(S.FIND_BANNER_FWD),                        # STUB: banner phase
        S.DESCEND_TO_CORRIDOR: altitude_state(S.DESCEND_TO_CORRIDOR, ALT_CORRIDOR),
        S.CORRIDOR_FORWARD: stub(S.CORRIDOR_FORWARD),                      # STUB: corridor phase
        S.CLIMB_TO_DELIVERY: altitude_state(S.CLIMB_TO_DELIVERY, ALT_DELIVERY),
        S.SEARCH_DELIVERY: h_search_delivery,
        S.CENTER_OVER_QR: h_center_over_qr,
        S.DESCEND_TO_DROP: altitude_state(S.DESCEND_TO_DROP, ALT_PAYLOAD_DROP, stay_over_target=True),
        S.DEPLOY_PAYLOAD: h_deploy_payload,
        S.CLIMB_AFTER_DROP: altitude_state(S.CLIMB_AFTER_DROP, ALT_DELIVERY),
        S.FIND_BANNER_RTN: stub(S.FIND_BANNER_RTN),                        # STUB: banner phase
        S.DESCEND_TO_CORRIDOR_RTN: altitude_state(S.DESCEND_TO_CORRIDOR_RTN, ALT_CORRIDOR),
        S.CORRIDOR_RETURN: stub(S.CORRIDOR_RETURN),                        # STUB: corridor phase
        S.RETURN_TO_HOME: h_return_to_home,
        S.LAND: h_land,
        S.EMERGENCY: h_emergency,
    }
