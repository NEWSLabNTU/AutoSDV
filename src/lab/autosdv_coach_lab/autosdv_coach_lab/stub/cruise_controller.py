"""Lab 2 headway law -- the OUTER loop. Students write this file.

The pursuit planner calls update() every planning cycle (10 Hz) with the
tracked coach board and the vehicle's speed, and you return the speed you want
the vehicle to drive at. That is the whole job: the planner turns your number
into a trajectory, and Autoware's controllers and your speed PID make the car
follow it.

You do NOT need to:
  - clamp to [0, v_max]          the planner does it, after you
  - limit acceleration           the planner does it, after you
  - stop at the standoff floor   the planner does it, after you, and always wins
  - handle a lost board          state.valid is False; the planner stops anyway

`state` fields (autosdv_lab_msgs/CoachState):
  valid       bool   False -> the board has not been seen recently
  range       m      base_link to board centre
  range_rate  m/s    positive = board pulling away
  bearing     rad    positive = left
  age         s      since the last detection behind this estimate

Parameters arrive in `params` from the planner's `controller.*` ROS
parameters, e.g. controller.headway:=2.0 -> params['headway'] == 2.0.
"""


class CruiseController:
    def __init__(self, params: dict):
        self.headway = float(params.get('headway', 2.0))
        # TODO: read your gains from params.
        self.reset()

    def reset(self) -> None:
        """Called when the board is (re)acquired and when autonomous mode is
        engaged. Clear any state that accumulates (integrators, filters)."""
        # TODO
        pass

    def update(self, state, ego_speed: float, dt: float) -> float:
        """Return the target speed in m/s. The shell clamps it."""
        if not state.valid:
            return 0.0
        # TODO: your headway law. Returning 0.0 keeps the car parked.
        return 0.0
