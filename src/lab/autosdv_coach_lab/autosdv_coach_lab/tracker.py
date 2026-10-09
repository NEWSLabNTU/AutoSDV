"""Coach board tracker: board detections in base_link -> range, range rate, bearing.

ROS-free. coach_tracker_node wraps it.

The board is tracked in the odometry frame, not in base_link. Each detection
(base_link, at its scan stamp) is placed in the odometry frame with the ego
pose *at that stamp*, and an alpha-beta filter estimates the board's own
position and velocity there. A walking TA is close to the constant-velocity
model the filter assumes; the board as seen from a vehicle that is
accelerating is not. Tracking in base_link made the relative velocity lag
every change in ego speed by the filter's time constant, and a headway law
that adds ego speed to that lagging range_rate (to recover the board's speed)
fed the vehicle's own acceleration back into its command: in the planning
simulator that was a sustained 0-1.5 m/s oscillation instead of following.

predict() then expresses the board relative to the ego pose and velocity *now*:

    range       distance base_link origin -> board, ground plane
    range_rate  d(range)/dt = (v_board - v_ego) . line of sight
    bearing     angle of the board in base_link, positive left

so ego speed enters range_rate exactly and instantly; only the board's own
velocity is filtered. Dead-reckoned odometry drifts, but only its change over
the last few hundred milliseconds matters here.

Without odometry the caller passes the identity pose and zero speed, and this
degrades to tracking in base_link.
"""

import bisect
import math
from dataclasses import dataclass


@dataclass
class TrackerParams:
    # Gains for a ~10 Hz detector. alpha weights position, beta velocity.
    alpha: float = 0.5
    beta: float = 0.3
    # No accepted detection for this long -> valid = False, and the next
    # detection re-initialises the track.
    timeout: float = 0.5
    # Innovation gate, metres. A detection further than this from the
    # prediction is rejected as an outlier...
    gate: float = 1.5
    # ...unless this many in a row are, in which case the board really moved
    # (or the first lock was wrong) and the track restarts on the new one.
    max_consecutive_rejects: int = 3
    # Bound on the board's estimated speed, m/s. A walking TA never exceeds it;
    # a filter that does is diverging.
    max_board_speed: float = 3.0
    # Detections closer / further than this are not the board.
    min_range: float = 0.3
    max_range: float = 30.0


@dataclass
class TrackedState:
    valid: bool
    range: float = 0.0
    range_rate: float = 0.0
    bearing: float = 0.0
    age: float = math.inf


def base_link_to_world(x, y, pose):
    """(x, y) in base_link -> odometry frame, for ego pose (ex, ey, eyaw)."""
    ex, ey, eyaw = pose
    c, s = math.cos(eyaw), math.sin(eyaw)
    return ex + c * x - s * y, ey + s * x + c * y


class PoseHistory:
    """Ego poses by time, for placing a lagging detection at its own stamp."""

    def __init__(self, horizon: float = 2.0):
        self.horizon = horizon
        self.t = []
        self.p = []

    def add(self, t, x, y, yaw):
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t)
        self.p.append((x, y, yaw))
        while self.t and self.t[0] < t - self.horizon:
            self.t.pop(0)
            self.p.pop(0)

    def __len__(self):
        return len(self.t)

    def at(self, t):
        """Pose at t, linearly interpolated; clamped to the ends; None if empty."""
        if not self.t:
            return None
        i = bisect.bisect_left(self.t, t)
        if i <= 0:
            return self.p[0]
        if i >= len(self.t):
            return self.p[-1]
        t0, t1 = self.t[i - 1], self.t[i]
        a = (t - t0) / (t1 - t0)
        (x0, y0, w0), (x1, y1, w1) = self.p[i - 1], self.p[i]
        dw = math.atan2(math.sin(w1 - w0), math.cos(w1 - w0))
        return x0 + a * (x1 - x0), y0 + a * (y1 - y0), w0 + a * dw


IDENTITY = (0.0, 0.0, 0.0)


class AlphaBetaTracker:
    def __init__(self, params: TrackerParams = None):
        self.p = params or TrackerParams()
        self.reset()

    def reset(self) -> None:
        self.initialised = False
        self.t = None          # stamp of the last accepted detection
        self.x = self.y = 0.0  # board, odometry frame
        self.vx = self.vy = 0.0
        self.rejects = 0

    def _start(self, t, x, y):
        self.initialised = True
        self.t = t
        self.x, self.y = x, y
        self.vx = self.vy = 0.0
        self.rejects = 0

    def update(self, t: float, x: float, y: float, ego_pose=IDENTITY) -> bool:
        """File a detection at stamp t (s): (x, y) in base_link, ego_pose the
        ego (x, y, yaw) in the odometry frame at t. True if accepted."""
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(t)):
            return False
        r = math.hypot(x, y)
        if r < self.p.min_range or r > self.p.max_range:
            return False
        wx, wy = base_link_to_world(x, y, ego_pose)
        if not self.initialised or (t - self.t) > self.p.timeout:
            self._start(t, wx, wy)
            return True
        dt = t - self.t
        if dt <= 1e-4:
            # Out of order or a duplicate stamp: nothing to learn from it.
            return False
        xp = self.x + self.vx * dt
        yp = self.y + self.vy * dt
        ex, ey = wx - xp, wy - yp
        if math.hypot(ex, ey) > self.p.gate:
            self.rejects += 1
            if self.rejects >= self.p.max_consecutive_rejects:
                self._start(t, wx, wy)
                return True
            return False
        self.rejects = 0
        self.x = xp + self.p.alpha * ex
        self.y = yp + self.p.alpha * ey
        self.vx += self.p.beta / dt * ex
        self.vy += self.p.beta / dt * ey
        s = math.hypot(self.vx, self.vy)
        if s > self.p.max_board_speed:
            k = self.p.max_board_speed / s
            self.vx *= k
            self.vy *= k
        self.t = t
        return True

    def predict(self, t: float, ego_pose=IDENTITY, ego_speed: float = 0.0) -> TrackedState:
        """The board relative to the ego at time t (s), given the ego's pose
        and forward speed at t."""
        if not self.initialised:
            return TrackedState(valid=False)
        age = t - self.t
        if age > self.p.timeout:
            return TrackedState(valid=False, age=age)
        dt = max(age, 0.0)
        bx = self.x + self.vx * dt
        by = self.y + self.vy * dt
        ex, ey, eyaw = ego_pose
        c, s = math.cos(eyaw), math.sin(eyaw)
        dx, dy = bx - ex, by - ey
        rng = math.hypot(dx, dy)
        rvx = self.vx - ego_speed * c
        rvy = self.vy - ego_speed * s
        rate = (dx * rvx + dy * rvy) / rng if rng > 1e-6 else 0.0
        # bearing in base_link
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        return TrackedState(
            valid=True, range=rng, range_rate=rate,
            bearing=math.atan2(ly, lx), age=dt)
