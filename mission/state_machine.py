# =============================================================================
# mission/state_machine.py — Team Vajra AeroTHON 2026
#
# The engine that flies the mission. Every tick (CONTROL_HZ) it:
#   1. asks the watchdog          -> pilot took over? battery low?
#   2. checks the mission clock   -> out of time? -> EMERGENCY (RTL)
#   3. checks the state's clock   -> state too slow? -> its fallback state
#   4. runs the state's handler   -> handler(ctx) returns None (keep going)
#                                    or the next State
#   5. rejects illegal jumps      -> anything not in allowed_next() = bug -> EMERGENCY
# Any exception inside a handler also ends in EMERGENCY, never a crash.
# =============================================================================
import traceback
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from config.params import (
    CONTROL_HZ, MISSION_TIME_LIMIT_S, MISSION_EMERGENCY_MARGIN_S, EMERGENCY_TIMEOUT_S,
)
from mission.states import State, TERMINAL, TIMEOUT_FALLBACK, allowed_next, timeout_for

Handler = Callable[[object], Optional[State]]

# States that already head home: the "out of time" emergency would change nothing.
_NO_TIME_EMERGENCY = {State.RETURN_TO_HOME, State.LAND, State.EMERGENCY, State.DONE}


@dataclass
class MissionResult:
    final_state: State
    success: bool                      # finished by landing, no emergency, no pilot takeover
    reason: str                        # "completed", "pilot_override ...", "low_battery ...", ...
    payload_released: bool
    elapsed_s: float
    history: List[Tuple[str, float]] = field(default_factory=list)   # (state, mission-time entered)


class StateMachine:
    def __init__(self, ctx, handlers: Dict[State, Handler]):
        missing = [s.name for s in State if s not in TERMINAL and s not in handlers]
        if missing:
            raise ValueError(f"No handler for states: {missing}")
        self.ctx = ctx
        self.handlers = handlers
        self.history: List[Tuple[str, float]] = []

    # ── helpers ───────────────────────────────────────────────────────────────
    def _enter(self, state: State):
        ctx = self.ctx
        ctx.state = state
        ctx.state_t0 = ctx.clock()
        ctx.sd = {}
        self.history.append((state.name, ctx.mission_elapsed()))
        ctx.log(f"→ {state.name}")

    def _emergency(self, reason: str) -> State:
        if "emergency_reason" not in self.ctx.data:
            self.ctx.data["emergency_reason"] = reason
        self.ctx.log(f"!! EMERGENCY: {reason}")
        return State.EMERGENCY

    def _time_limit_for(self, state: State) -> Optional[float]:
        if state is State.EMERGENCY:
            return EMERGENCY_TIMEOUT_S
        return timeout_for(state)

    def _result(self, final: State, reason: Optional[str] = None) -> MissionResult:
        ctx = self.ctx
        bad = ctx.data.get("emergency_reason") or ctx.data.get("stop_reason")
        reason = reason or bad or "completed"
        return MissionResult(final, success=(bad is None), reason=reason,
                             payload_released=ctx.payload_released,
                             elapsed_s=ctx.mission_elapsed(), history=list(self.history))

    # ── main loop ─────────────────────────────────────────────────────────────
    def run(self, start: State = State.INIT) -> MissionResult:
        ctx = self.ctx
        period = 1.0 / CONTROL_HZ
        state = start
        self._enter(state)

        while state not in TERMINAL:
            nxt: Optional[State] = None

            # 1. watchdog
            ev = ctx.safety.check() if ctx.safety is not None else None
            if ev is not None:
                if ev.action == "stop":                 # pilot took over: hands off!
                    ctx.data["stop_reason"] = ev.reason
                    ctx.log(f"!! PILOT OVERRIDE ({ev.reason}) — no more commands")
                    return self._result(state, ev.reason)
                if state is not State.EMERGENCY:
                    nxt = self._emergency(ev.reason)

            # 2. mission clock
            if (nxt is None and state not in _NO_TIME_EMERGENCY and ctx.mission_t0 is not None
                    and ctx.mission_elapsed() >= MISSION_TIME_LIMIT_S - MISSION_EMERGENCY_MARGIN_S):
                nxt = self._emergency(f"mission_time ({ctx.mission_elapsed():.0f} s)")

            # 3. state clock
            limit = self._time_limit_for(state)
            if nxt is None and limit is not None and ctx.state_age() > limit:
                nxt = State.DONE if state is State.EMERGENCY else TIMEOUT_FALLBACK.get(state, State.EMERGENCY)
                ctx.log(f"   {state.name} timed out after {limit:.0f} s → {nxt.name}")

            # 4. handler
            if nxt is None:
                try:
                    nxt = self.handlers[state](ctx)
                except Exception as exc:                 # noqa: BLE001 (we want everything)
                    ctx.log("!! handler crashed:\n" + traceback.format_exc())
                    if state is State.EMERGENCY:
                        nxt = State.DONE
                    else:
                        nxt = self._emergency(f"exception in {state.name}: {exc!r}")

            # 5. transition
            if nxt is None or nxt is state:
                ctx.sleep(period)
                continue
            if nxt not in allowed_next(state) and not (state is State.EMERGENCY and nxt is State.DONE):
                nxt = self._emergency(f"illegal transition {state.name} → {nxt.name}")
            state = nxt
            self._enter(state)
            if state in TERMINAL:
                break

        return self._result(state)
