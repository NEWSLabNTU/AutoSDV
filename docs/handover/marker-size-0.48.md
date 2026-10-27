# Handover: marker size 0.4667 -> 0.48 m (unfinished)

Branch `wip/marker-size-0.48`, from develop 68c399d. Not merged.

## Why

The user measured the lab's black ArUco square at **0.48 m** (600 mm board),
2026-10-27. The old default 0.4667 assumed 7 of 9 cells and was never measured.
A wrong `marker_size` scales every marker distance, so maps and fixes are off
by 0.48 / 0.4667 = 1.029.

## Done on this branch (uncommitted on the car, committed here)

- Defaults to 0.48: `marker_survey_node.py`, `marker_survey.launch.xml`
  (the user asked for the default values; this is the only launch-file line
  touched), `synthetic_bag.py`, `marker_map.py` (doc example),
  `mock_camera_node.py` (PLATE_RATIO = 0.60 / 0.48), `aruco_map.sh`,
  `autosdv_marker_survey/README.md`, `docs/guides/aruco-map.md`,
  `docs/guides/lab2-waypoint-stack.md`.
- Courses: `marker_size: 0.48` in `lab2_uturn`, `lab2_straight_2m`,
  `lab2_straight_3m`, `lab2_quarter_r2` `course.yaml`; their
  `ar_tag.param.yaml` and `lanelet2_map.osm` regenerated with
  `just lab2 course-map`. (That command also rewrote `lab2_part1` and created its
  `ar_tag.param.yaml`; both reverted as unrelated.)
- `test_course.py`: 0.4667 -> 0.48.

## Remaining

1. **`uturn.py`: split the marker size in two.** The user chose to KEEP the
   shipped lab2_uturn board plan. The coverage search (`CameraModel(marker_size=...)`
   at lines ~367, 402, 488) must stay at 0.4667, because at 0.48 it moves one
   board (facing +150 deg -> +120 deg). The written course (`marker_size:` at
   ~626 and the `Course(...)` at ~433) and the header comments (~24, 591, 597,
   628, 691) must say 0.48. Suggested: `PLAN_MARKER_SIZE = 0.4667` (search
   only, commented as a planning value) and `MARKER_SIZE = 0.48`. Currently the
   branch has MARKER_SIZE = 0.48 everywhere, which is wrong for the search.
2. Then `src/lab/autosdv_waypoint_lab`: `test_course.py::
   test_lab2_uturn_shipped_files_are_what_make_uturn_writes` must pass with the
   shipped `lab2_uturn/course.yaml` (which already says 0.48). Do NOT run
   `make-uturn --write` with the search at 0.48.
3. Commit, rebase on develop, push develop.
4. **Re-solve existing surveys** taken at 0.4667: `just aruco-map solve DIR
   marker_size:=0.48`, then re-align (`just aruco-map align` or
   `just lab2 survey-align`). Car A's `surveys/2026-10-27_18-15-13` (car B's
   survey, marker_size 0.4667) needs this before the next run; car B's and
   car C's own surveys too.

## Test failures that are NOT this change

- `autosdv_waypoint_lab/test/test_steer_cal.py::test_cli_fit_exit_codes`: reads
  car B's live profile, whose steering ends changed today (330..470).
- `autosdv_marker_survey/test/test_survey.py::...without_arucodetector`: this
  Jetson's cv2 has no `cv2.aruco.DetectorParameters`.

## Other open items from the car A session (2026-10-27)

- Wheel speed over-reads: car A 2.1x, car B 1.9x (pin 16), car C 1.3x on pin 15
  and 1.46x on pin 16, measured as wheel distance / marker-fix path. Hand test:
  one wheel revolution with `scripts/control/gpio_read.py`, `EDGE = GPIO.RISING`
  then `GPIO.BOTH` (expect 12 / 24). Then measure the wheel circumference.
  Do not change `wheel_diameter` / `markers_per_rotation` before that.
- Car A steering ratio is -249 (measured full-lock circle). `steer_cal fit`
  still fits on the bicycle angle, about 2 % off the actuator's average angle.
