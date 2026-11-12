# Phase 6: Real car, grip

**Status:** planned. **Needs:** phase 2 (measured specs) and phase 5 (the estimator on the
car).

[← Phase 5](5-cameras-and-estimator.md) · [Overview](README.md) · [Phase 7 →](7-real-car-drift.md)

## Goal

The race stack drives grip laps on the real cars on COSS, under a cap the operator raises
in steps, with all four cap layers live.

## Vehicle interface (AutoSDV)

- **6.1 Acceleration pass-through.** The actuator passes `longitudinal.acceleration` to the
  speed-controller plug-in; `AckermannSpeedPID` ignores it, as today, and its tests are
  unchanged.
- **6.2 Accel→ESC plug-in**, selected only by the race launch: acceleration to ESC PWM
  through the ESC model (`docs/design/esc-pwm-model.md`) and the car's coast-down fit, with
  feedback on wheel speed. Bench-tested, then on the ground with the race stack's speed
  profile.
- **6.3 Overspeed guard**, off by default (`max_speed: 0`): wheel speed above cap + margin →
  coast, then brake, with a diagnostic. Margin and brake profile: builder's call, within the
  vehicle interface's longitudinal shell.
- **6.4 Per-test `max_pwm`** override (today 470 on every car) set from `speed_cap:=` as a
  physical throttle ceiling.
- **6.5** The race diagnostic graph adds overspeed (guard), leaving the corridor, and a spin
  (sideslip or yaw rate outside the recoverable envelope, from the controller) as MRM
  faults.

## Procedure on COSS

- A written test card per session: car, cap, laps, spotter with the RC transmitter's
  override and the e-stop, weather and surface.
- Car A first: cap 1.0 m/s, warm-up lap for rig, then laps; raise the cap in steps only
  after a set of clean laps (no MRM, tracking error inside the corridor) at the current one.
- Cars B and C on the same card once A has run its steps.

## Exit

- Each car: clean grip laps on the COSS course at a stated cap; bags, lap times and
  tracking error against the plan, per cap step, in the phase's report.
- The overspeed guard shown to act on the car (a cap set below the planned speed on a
  straight; the guard coasts and brakes, and the diagnostic fires).
- `just launch` on each car without the race recipes is unchanged (the plug-in and guard
  are inactive by default).
