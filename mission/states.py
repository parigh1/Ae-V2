# =============================================================================
# mission/states.py — Team Vajra AeroTHON 2026
#
# The 22 states of the SkyScan mission, in flying order, plus what to do when a
# state runs out of time.  Rulebook section 4.2.4 / report section 4.1.
# =============================================================================

from enum import Enum

from config.params import STATE_TIMEOUTS


class State(Enum):
    INIT = "INIT"                                    # connect, set up
    PREFLIGHT = "PREFLIGHT"                          # battery, GPS, failsafe params
    ARM = "ARM"                                      # GUIDED mode + arm
    TAKEOFF = "TAKEOFF"                              # climb to 5 m  (mission clock starts)
    MOVE_TO_QR_POINT = "MOVE_TO_QR_POINT"            # ~1 m forward (rulebook)
    SCAN_START_QR = "SCAN_START_QR"                  # read the delivery ID
    FIND_BANNER_FWD = "FIND_BANNER_FWD"              # green banner, align the nose
    DESCEND_TO_CORRIDOR = "DESCEND_TO_CORRIDOR"      # down to 3 m (10 ft)
    CORRIDOR_FORWARD = "CORRIDOR_FORWARD"            # 3.5 m wide corridor, outbound
    CLIMB_TO_DELIVERY = "CLIMB_TO_DELIVERY"          # up to 10 m
    SEARCH_DELIVERY = "SEARCH_DELIVERY"              # lawnmower, avoid red zones
    CENTER_OVER_QR = "CENTER_OVER_QR"                # visual servo onto the right QR
    DESCEND_TO_DROP = "DESCEND_TO_DROP"              # down to 5 m
    DEPLOY_PAYLOAD = "DEPLOY_PAYLOAD"                # release
    CLIMB_AFTER_DROP = "CLIMB_AFTER_DROP"            # back up to 10 m
    FIND_BANNER_RTN = "FIND_BANNER_RTN"              # banner again, return entrance
    DESCEND_TO_CORRIDOR_RTN = "DESCEND_TO_CORRIDOR_RTN"
    CORRIDOR_RETURN = "CORRIDOR_RETURN"              # corridor, inbound
    RETURN_TO_HOME = "RETURN_TO_HOME"                # GPS back over the start point
    LAND = "LAND"
    DONE = "DONE"                                    # finished (terminal)
    EMERGENCY = "EMERGENCY"                          # RTL / abort, from any state


# Normal flying order (EMERGENCY is entered from anywhere, so it is not in the chain)
ORDER = [s for s in State if s is not State.EMERGENCY]
NEXT = {a: b for a, b in zip(ORDER, ORDER[1:])}

TERMINAL = {State.DONE}

# When a state runs out of time, jump HERE instead of the normal next state.
# Idea: keep earning partial marks where it is safe; bail out where it is not.
TIMEOUT_FALLBACK = {
    State.INIT: State.EMERGENCY,
    State.PREFLIGHT: State.EMERGENCY,
    State.ARM: State.EMERGENCY,
    State.TAKEOFF: State.EMERGENCY,
    State.MOVE_TO_QR_POINT: State.SCAN_START_QR,
    State.SCAN_START_QR: State.FIND_BANNER_FWD,        # no ID -> can't pick a target, but fly the corridor
    State.FIND_BANNER_FWD: State.EMERGENCY,            # not aligned -> don't fly into the walls
    State.DESCEND_TO_CORRIDOR: State.CORRIDOR_FORWARD,
    State.CORRIDOR_FORWARD: State.EMERGENCY,
    State.CLIMB_TO_DELIVERY: State.SEARCH_DELIVERY,
    State.SEARCH_DELIVERY: State.FIND_BANNER_RTN,      # target not found -> skip the drop, go home
    State.CENTER_OVER_QR: State.DESCEND_TO_DROP,       # not perfectly centred -> drop anyway
    State.DESCEND_TO_DROP: State.DEPLOY_PAYLOAD,
    State.DEPLOY_PAYLOAD: State.CLIMB_AFTER_DROP,
    State.CLIMB_AFTER_DROP: State.FIND_BANNER_RTN,
    State.FIND_BANNER_RTN: State.EMERGENCY,
    State.DESCEND_TO_CORRIDOR_RTN: State.CORRIDOR_RETURN,
    State.CORRIDOR_RETURN: State.EMERGENCY,
    State.RETURN_TO_HOME: State.LAND,
    State.LAND: State.DONE,
}


def timeout_for(state: State) -> float:
    """Seconds this state may take (None = no limit, e.g. DONE / EMERGENCY)."""
    return STATE_TIMEOUTS.get(state.value)


# Moves that are legal but are neither "next" nor a timeout fallback.
EXTRA_ALLOWED = {
    State.CENTER_OVER_QR: {State.SEARCH_DELIVERY, State.FIND_BANNER_RTN},  # lost the target
}


def allowed_next(state: State) -> set:
    """Every state the machine may move to from `state` (anything else is a bug)."""
    allowed = {State.EMERGENCY}
    if state in NEXT:
        allowed.add(NEXT[state])
    if state in TIMEOUT_FALLBACK:
        allowed.add(TIMEOUT_FALLBACK[state])
    allowed |= EXTRA_ALLOWED.get(state, set())
    return allowed
