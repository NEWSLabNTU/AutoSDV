"""Reference headway law: proportional on headway error, plus feed-forward of
the board's own speed. TA copy -- do not ship to students.

    v_board  = ego_speed + range_rate        (range_rate > 0: board pulls away)
    v_target = k_ff * v_board + kp * (range - headway)

Why the feed-forward: a P-only law needs a standing headway error to hold any
speed at all (v = kp * e), so following a board at 0.8 m/s with kp = 0.6 sits
1.33 m behind the target headway -- measured 3.33 m against 2.0 m in the
planning simulator. Feeding the board's speed forward supplies that speed
directly and leaves the P term only the error to remove, so the steady-state
error goes to zero without an integrator to wind up.

The shell (pursuit.clamp_speed) clamps the result to [0, v_max], rate-limits
it, and enforces the standoff floor, so this class does not.
"""

import math


class CruiseController:
    def __init__(self, params: dict):
        self.headway = float(params.get('headway', 2.0))
        self.kp = float(params.get('kp', 0.6))
        self.k_ff = float(params.get('k_ff', 1.0))
        # Low-pass on the board-speed estimate; range_rate is a differentiated
        # signal and carries the detector's noise.
        self.ff_tau = float(params.get('ff_tau', 0.3))
        self.reset()

    def reset(self) -> None:
        self.v_board = None

    def update(self, state, ego_speed: float, dt: float) -> float:
        if not state.valid:
            self.reset()
            return 0.0
        v_board = max(ego_speed + state.range_rate, 0.0)
        if self.v_board is None or self.ff_tau <= 0.0:
            self.v_board = v_board
        else:
            a = 1.0 - math.exp(-max(dt, 0.0) / self.ff_tau)
            self.v_board += a * (v_board - self.v_board)
        error = state.range - self.headway
        return self.k_ff * self.v_board + self.kp * error
