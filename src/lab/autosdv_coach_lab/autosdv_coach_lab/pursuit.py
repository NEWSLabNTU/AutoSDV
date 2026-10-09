"""The pursuit planner shell: everything around the student's CruiseController.

ROS-free. board_pursuit_planner_node wraps it.

What the shell owns, so that no student gain can break it:

- loading the controller by import path;
- the clamps: 0 <= v <= v_max, an acceleration and deceleration rate limit,
  and a braking envelope that reaches zero at the standoff floor;
- the stop when the coach state is invalid;
- the trajectory: a FIXED-LENGTH straight path toward the board, with the stop
  encoded in velocity: a braking ramp that reaches zero at s_stop, and zero
  from there on.

Two decelerations, deliberately different. brake_decel is what the
trajectory PLANS: the ramps are drawn at it, and their accelerations reach
Autoware's longitudinal PID as feed-forward, so it is very nearly the braking
the vehicle then does (the profile is re-anchored at the current speed every
cycle, so there is no velocity error left for the PID's feedback to add to it).
envelope_decel is what the SAFETY envelope assumes the vehicle can be relied
on for, plus reaction_time for detector, planner and controller latency; it is
lower, and the gap between the two is the margin. Autoware's longitudinal PID only
"sees" a stop point within ~0.5 m of it (drive_state_stop_dist) and then
brakes through smooth_stop at -0.5 to -0.8 m/s^2; a trajectory that holds full
speed up to a velocity step leaves it no room, and in the planning simulator
the vehicle ran 1 m past the floor into the board. With the ramp the PID tracks
a decelerating target in its DRIVE state, as it does behind Autoware's own
velocity smoother.

The last point is load-bearing. A path that shrinks as the vehicle nears the
standoff crashes Autoware's MPC ("Either base_keys or query_keys is not
sorted", SIGABRT, then MRM). Geometry never changes length; only the velocity
profile says where to stop.
"""

import importlib
import math
from dataclasses import dataclass


@dataclass
class PlannerParams:
    v_max: float = 1.5           # m/s, hard ceiling on the commanded speed
    accel_max: float = 0.5       # m/s^2, how fast the command may rise
    decel_max: float = 2.0       # m/s^2, how fast it may fall
    brake_decel: float = 1.0     # m/s^2, braking the trajectory plans (ramps)
    envelope_decel: float = 0.6  # m/s^2, braking the safety envelope relies on
    reaction_time: float = 0.5   # s, sensing-to-braking latency the envelope allows
    standoff_min: float = 1.0    # m, board range at which speed must be zero
    path_length: float = 10.0    # m, fixed
    path_step: float = 0.2       # m between trajectory points


# ---------------------------------------------------------------------------
# Controller loading
# ---------------------------------------------------------------------------

def load_controller(spec: str, params: dict):
    """Instantiate `package.module:ClassName` with a params dict.

    The class must provide reset() and update(state, ego_speed, dt). Raises
    ValueError for a malformed spec or an incomplete class, ImportError /
    AttributeError when the module or class is missing.
    """
    module_name, sep, class_name = (spec or '').partition(':')
    if not sep or not module_name or not class_name:
        raise ValueError(
            f"controller_class must be 'package.module:ClassName', got {spec!r}")
    module = importlib.import_module(module_name)
    cls = getattr(module, class_name)
    obj = cls(dict(params))
    for method in ('reset', 'update'):
        if not callable(getattr(obj, method, None)):
            raise ValueError(f'{spec} has no {method}() method')
    return obj


# ---------------------------------------------------------------------------
# Clamps
# ---------------------------------------------------------------------------

def clamp_speed(v_raw, v_prev, dt, rng, valid, p: PlannerParams):
    """Turn the controller's request into the commanded speed.

    Returns (v_command, limit), where limit names the clamp that bound last
    ('' when none did). Order matters: the rate limit smooths the request, then
    the safety caps (v_max, the standoff envelope) are applied on top, so they
    always win over the rate limit.
    """
    if not valid:
        return 0.0, 'invalid'
    if v_raw is None or not math.isfinite(v_raw):
        return 0.0, 'error'
    limit = ''
    v = max(v_raw, 0.0)
    dt = max(dt, 0.0)
    if v > v_prev + p.accel_max * dt:
        v = v_prev + p.accel_max * dt
        limit = 'accel'
    elif v < v_prev - p.decel_max * dt:
        v = max(v_prev - p.decel_max * dt, 0.0)
        limit = 'decel'
    if v > p.v_max:
        v = p.v_max
        limit = 'v_max'
    gap = rng - p.standoff_min
    if gap <= 0.0:
        return 0.0, 'standoff'
    v_env = braking_speed(gap, p)
    if v > v_env:
        v = v_env
        limit = 'standoff'
    return v, limit


def braking_speed(gap, p: PlannerParams) -> float:
    """Largest speed from which the vehicle stops within `gap` metres, after
    reaction_time at that speed and then envelope_decel: solves
    v * t_r + v^2 / (2 a) = gap."""
    if gap <= 0.0:
        return 0.0
    a, tr = p.envelope_decel, p.reaction_time
    return a * (math.sqrt(tr * tr + 2.0 * gap / a) - tr)


def stop_distance(v_command, v_ego, rng, valid, p: PlannerParams) -> float:
    """Arc length at which the velocity profile reaches zero.

    Driving (v_command > 0): the standoff floor, so the ramp in front of it is
    always in the trajectory. Stopping (v_command == 0, or the board lost):
    the distance the vehicle needs at brake_decel from its current speed, but
    never past the floor when the board is known. A stationary vehicle gets 0,
    an all-zero profile that keeps it stopped.
    """
    brake = max(v_ego, 0.0) ** 2 / (2.0 * p.brake_decel)
    if not valid:
        return min(brake, p.path_length)
    gap = min(max(rng - p.standoff_min, 0.0), p.path_length)
    if v_command <= 0.0:
        return min(brake, gap)
    return gap


# ---------------------------------------------------------------------------
# Trajectory
# ---------------------------------------------------------------------------

def point_count(p: PlannerParams) -> int:
    return int(round(p.path_length / p.path_step)) + 1


def velocity_profile(v_command, v_ego, s_stop, p: PlannerParams):
    """Target speed and acceleration at each trajectory point.

    Three caps, each non-increasing in s, so their minimum is too:
      v_command                              the clamped command;
      max(v_command, sqrt(v_ego^2 - 2 a s))  when the command is below the
                                             current speed, a ramp down to it
                                             from the current speed at a, so
                                             the controller brakes along a
                                             target instead of at a step;
      sqrt(2 a_stop (s_stop - s))            the ramp to zero at s_stop, then 0.
    a = brake_decel; a_stop is brake_decel, or harder if the vehicle cannot
    otherwise stop within s_stop. Accelerations are the profile's own
    (v_{i+1}^2 - v_i^2) / (2 ds): Autoware's longitudinal PID uses them as
    feed-forward, which is what makes it brake on the ramp rather than only
    on the velocity error.
    """
    n = point_count(p)
    ds = p.path_step
    a = p.brake_decel
    v_ego = max(v_ego, 0.0)
    a_stop = a
    if s_stop > 1e-9:
        a_stop = max(a, v_ego * v_ego / (2.0 * s_stop))
    vs = []
    for i in range(n):
        d = i * ds
        if d >= s_stop - 1e-9:
            vs.append(0.0)
            continue
        v = v_command
        if v_ego > v_command:
            v = max(v_command, math.sqrt(max(v_ego * v_ego - 2.0 * a * d, 0.0)))
        v = min(v, math.sqrt(2.0 * a_stop * (s_stop - d)))
        vs.append(v)
    accs = [(vs[i + 1] ** 2 - vs[i] ** 2) / (2.0 * ds) for i in range(n - 1)]
    accs.append(0.0)
    return vs, accs


def build_trajectory(x0, y0, heading, v_command, s_stop, p: PlannerParams, v_ego=0.0):
    """The fixed-length straight path from the ego toward `heading`.

    Returns a list of (x, y, yaw, v, acc) tuples, always point_count(p) long,
    with arc length increasing by path_step; v and acc from velocity_profile.
    The geometry never depends on the stop: only the velocities do.
    """
    c, s = math.cos(heading), math.sin(heading)
    vs, accs = velocity_profile(v_command, v_ego, s_stop, p)
    return [(x0 + i * p.path_step * c, y0 + i * p.path_step * s, heading, v, acc)
            for i, (v, acc) in enumerate(zip(vs, accs))]


def yaw_from_quaternion(qx, qy, qz, qw) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
