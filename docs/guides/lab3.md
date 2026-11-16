# Lab 3: indoor map building and NDT localization

The AutoSDV side of Lab 3: a team drives car B or C (Seyond Robin-W + ZED X Mini IMU)
around the classroom, builds a point-cloud map with GLIM, draws a clockwise lanelet2 loop
in Vector Map Builder, and in week 2 drives that loop autonomously on NDT, started and
sent round the loop from RViz the standard Autoware way. Package `src/lab/autosdv_lab3`.
Students use stock ROS commands (below); `just lab3` (`just/lab3.just`) holds the TA's
shortcuts. The lab's plan is in the course repository, `2026/labs/Lab3/`.

**The map frame is GLIM's.** There is no anchor: the map's origin is where the car stood
when the mapping recording began, so the team starts that recording with the car on the
taped start mark, and the origin is roughly the mark. Each team's origin differs a
little, and nothing assumes a fixed one. Measured on the TA simulation's mapping bag:
the origin is the IMU's position at the first scans, z is up (gravity), and the
horizontal axes are the LiDAR's own at that moment. The Robin-W's frame is x up, y right,
z forward, so the map's **+x points backward** from the car's starting heading and +y to
its right; the floor lies about the IMU's height below z = 0. This matters only for
reading coordinates: the start is set in RViz, and the loop's direction is checked with z
up.

## Student commands

Every command below was run as written (see "Verified" for how), except that the two
on-car launches (mapping, run) and the grader were only resolved (`play_launch dump`), not
run on a car; `team3`, `car_b` and the bag names are examples. Each terminal first sources the AutoSDV workspace
(`source ~/repos/AutoSDV/scripts/env.sh`, or wherever the car's checkout is). The topic
lists live in the installed package; a team may copy them, e.g. into `~/lab3/team3/`, and
`cat` its copy instead.

### Week 1: the mapping drive (on the car)

Terminal 1, the sensors (LiDAR, ZED IMU, wheel speed, TF; the actuator never drives:
steer with the transmitter):

```bash
ros2 launch autosdv_lab3 lab3_mapping.launch.xml vehicle_id:=car_b
```

Terminal 2, the recording. Put the car on the start mark first: where the recording
begins is the map's origin. Keep it still 10 s, drive the room at no more than 0.8 m/s
closing the loop twice, stop, keep it still 10 s, then Ctrl-C:

```bash
mkdir -p ~/lab3/bags && cd ~/lab3/bags && ros2 bag record $(cat $(ros2 pkg prefix autosdv_lab3)/share/autosdv_lab3/config/record/map_topics.txt)
```

The bag is `~/lab3/bags/rosbag2_YYYY_MM_DD-HH_MM_SS`. Check it before leaving the hall
(LiDAR fields, IMU rate, TF, duration, speed, standing still at both ends):

```bash
ros2 run autosdv_lab3 lab3_check_bag ~/lab3/bags/rosbag2_2026_11_20-10_15_00
```

### Week 1: the map (anywhere with GLIM; stop the mapping launch first)

GLIM, headless, with the ready config of the Lab 3 cars (`robin_w_ready`, one for cars B
and C, which carry the same sensor set; `robin_w_ready_cpu` on a computer without an
NVIDIA GPU):

```bash
ros2 run glim_ros glim_rosbag ~/lab3/bags/rosbag2_2026_11_20-10_15_00 --ros-args -p config_path:=$(ros2 pkg prefix autosdv_lab3)/share/autosdv_lab3/config/glim/robin_w_ready -p dump_path:=$HOME/lab3/team3_glim -p auto_quit:=true
```

It plays the bag (a 196 s bag took 155 s on a workstation GPU), optimizes, writes the dump
directory `~/lab3/team3_glim` and quits.

Export the points with GLIM's viewer:

```bash
ros2 run glim_ros offline_viewer
```

1. **File > Open New Map**; in the folder dialog type the dump path
   (`~/lab3/team3_glim`, Ctrl+L opens the path field) and press **OK**.
2. "Do optimization?": **No** (the dump is already optimized). The map appears.
3. **File > Save > Export Points**. The **Name** field starts as `.`: clear it, type
   `team3_map.ply`, pick the folder `~/lab3`, **OK**. (Typing over the pre-selected part
   leaves the `.` and writes `team3_map.ply.`.) The viewer logs "exporting points"; the
   file is binary PLY with x y z intensity.
4. **File > Quit**.

On a VNC desktop whose OpenGL does not reach the GPU, start it as
`vglrun -d egl ros2 run glim_ros offline_viewer`.

To an Autoware map, downsampled to 0.1 m (stock PCL; the last step writes plain binary,
since `pcl_voxel_grid` writes `binary_compressed`):

```bash
mkdir -p ~/lab3/team3_map
pcl_ply2pcd ~/lab3/team3_map.ply ~/lab3/team3_raw.pcd
pcl_voxel_grid ~/lab3/team3_raw.pcd ~/lab3/team3_voxel.pcd -leaf 0.1,0.1,0.1
pcl_convert_pcd_ascii_binary ~/lab3/team3_voxel.pcd ~/lab3/team3_map/pointcloud_map.pcd 1
echo "projector_type: Local" > ~/lab3/team3_map/map_projector_info.yaml
```

Draw the loop in Vector Map Builder on `pointcloud_map.pcd` and export
`lanelet2_map.osm` into the same folder. The folder is then:

```
~/lab3/team3_map/
    pointcloud_map.pcd          GLIM's map, 0.1 m voxels
    lanelet2_map.osm            the clockwise loop (Local projector: local_x/local_y tags)
    map_projector_info.yaml     projector_type: Local
```

Draw the loop with a lanelet boundary **just behind the start mark** (the origin): in
week 2 the one goal goes about 1.0 m behind the start and must fall in the lanelet before
the start's (see "The one-goal lap").

### Week 1: check the loop

```bash
ros2 run autosdv_lab3 lab3_check_lanelet ~/lab3/team3_map
```

Four requirements, PASS or FAIL: the road lanelets form **one closed loop** by successor
relations, it runs **clockwise** seen from above, every lanelet has a **speed limit**
above 0 and at most 10 km/h, and a **start boundary**: a lanelet boundary 0.4-0.8 m behind
the map origin (aim 0.5 m), the origin no more than 1 m off the loop. A hint below the
table says where the boundary and a goal 1.0 m behind the start fall. Then, unless `--no-sim`, one lap in the planning simulator
with the field procedure: start at the origin projected onto the loop, one goal 1.0 m
behind it (`--goal-behind` to change). Set `ROS_DOMAIN_ID` first when the computer is
shared.

### Practise: the planning simulator (anywhere)

```bash
ros2 launch autosdv_lab3 lab3_planning_sim.launch.xml map_path:=$HOME/lab3/team3_map
```

AutoSDV's RC simulator on the team's map with the car's planning settings
(`planning_profile:=rc`; Autoware's stock `planning_simulator.launch.xml` uses the stock
planning files, with which a one-goal lap does not drive). RViz opens; the car appears at
the origin facing the map's +x (backward, see above). Then the RViz sequence below. The
overall speed limit is the same file as on the car (`speed_limit_config:=` to use a
team's copy).

### Part 1: replay the mapping bag against the map (localization report)

The team plays its own mapping bag back through the stack (no drivers, no vehicle
interface, simulated time), localizing against the map built from it, and hands in the
localization report. Three terminals, in this order:

```bash
SHARE=$(ros2 pkg prefix autosdv_lab3)/share/autosdv_lab3
# 1. the stack, sensors and motor off
play_launch launch autosdv_lab3 lab3_replay.launch.xml vehicle_id:=car_b map_path:=$HOME/lab3/team3_map
# 2. record the localization output (stops with Ctrl-C after the bag ends)
mkdir -p ~/lab3/bags && cd ~/lab3/bags && ros2 bag record $(cat $SHARE/config/record/localization_topics.txt)
# 3. play the mapping bag's sensor inputs
ros2 bag play ~/lab3/bags/<mapping bag> --clock --topics $(cat $SHARE/config/record/replay_topics.txt)
```

In RViz, a 2D Pose Estimate at the map origin within the first seconds, while the car
still stands on the start mark: the mapping recording began there. On the Robin-W cars
GLIM's map +x points backward from the start heading, so the car faces the map's -x.
After the bag ends, Ctrl-C the recorder, then

```bash
ros2 run autosdv_lab3 lab3_localization_report ~/lab3/bags/<replay bag>
```

prints and writes `localization.yaml`: the localized share (0.5 s bins with NVTL at or
above the 1.3 gate), NVTL median and minimum, and the scan-to-map distance (one scan per
0.5 s placed at the estimated pose, median distance to the nearest map point, share of
points on the map). `--map PCD` supplies the map when a recording lacks
`/map/pointcloud_map`. The grader reports the same measures over a lap.

### Week 2: the laps (on the car)

Terminal 1, the stack (no RViz: the ZED must not see a display, so the launch removes
`DISPLAY` from its own environment):

```bash
ros2 launch autosdv_lab3 lab3_run.launch.xml vehicle_id:=car_b map_path:=$HOME/lab3/team3_map
```

Terminal 2, RViz:

```bash
rviz2 -d $(ros2 pkg prefix autosdv_launch)/share/autosdv_launch/rviz/autosdv.rviz --ros-args --params-file $(ros2 pkg prefix autosdv_vehicle_description)/share/autosdv_vehicle_description/config/vehicle_info.param.yaml
```

Terminal 3, the grader (the TA starts it after the stack is up, before the goal):

```bash
ros2 launch autosdv_lab3 lab3_grader.launch.xml team:=3
```

**The RViz sequence**, on the car and in the simulator alike:

1. Place the car on the start mark.
2. **2D Pose Estimate**: click the start, drag the way the car faces. Wait for NDT to
   settle (the car's model sits on the map, the scan on the walls).
3. **2D Goal Pose**: click about 1.0 m behind the start on the loop (0.3 m further back if the car does not move after Auto), in the lanelet
   before the start's, drag the way of travel.
4. In the **AutowareStatePanel** (left), press **Auto**. The car drives the whole loop
   and stops at the goal.
5. Ctrl-C the grader after the car has stopped: it grades the bag and prints the result.

The grade reads only the bag (`ros2 run autosdv_lab3 lab3_grade BAG` repeats it):
travel time, travel distance, every lanelet driven, and the straight-line gap from the
start to where the car came to rest (behind or past the start; an overshoot is fine)
added to the distance. The lap must end no more than 2 m behind the start, or pass it.

**The speed limit.** The overall limit is a YAML file read by `lab3_speed_limit`
(3 km/h as shipped, at most 10); on each lanelet the car drives at no more than the lower
of it and the lanelet's own `speed_limit`. Copy it, edit `max_speed_kmh` and save: the
stack applies the change within a second, no restart:

```bash
cp $(ros2 pkg prefix autosdv_lab3)/share/autosdv_lab3/config/speed_limit.yaml ~/lab3/team3/speed_limit.yaml
ros2 launch autosdv_lab3 lab3_run.launch.xml vehicle_id:=car_b map_path:=$HOME/lab3/team3_map speed_limit_config:=$HOME/lab3/team3/speed_limit.yaml
```

Without `speed_limit_config:=` the stack reads the shared file in the workspace.

### At home: replay a lap against the map

The grader's bag (`~/lab3/bags/team3_run_<date>_<time>`) holds the sensors, or a team
records its own beside the stack:

```bash
mkdir -p ~/lab3/bags && cd ~/lab3/bags && ros2 bag record $(cat $(ros2 pkg prefix autosdv_lab3)/share/autosdv_lab3/config/record/run_topics.txt)
```

At home, terminal 1, the stack with no drivers and no vehicle interface, on the bag's
clock, RViz included:

```bash
ros2 launch autosdv_lab3 lab3_replay.launch.xml vehicle_id:=car_b map_path:=$HOME/lab3/team3_map
```

Terminal 2, play only the raw sensor inputs (the stack makes the rest, TF included):

```bash
ros2 bag play ~/lab3/bags/team3_run_20261127_101500 --clock --topics $(cat $(ros2 pkg prefix autosdv_lab3)/share/autosdv_lab3/config/record/replay_topics.txt)
```

After the first 3 s and while the car still stands, **2D Pose Estimate** where it
stood (Autoware refuses an initial pose unless the wheel speed has been zero for 3 s:
"The vehicle is not stopped"; once the car moves, restart both terminals), and watch NDT
follow the drive (`/localization/pose_estimator/nearest_voxel_transformation_likelihood`
in PlotJuggler or `ros2 topic echo`).

## The one-goal lap

With `planning_profile:=rc` (`autosdv_launch/config/planning_rc/README.md`), one goal just
behind the start drives the whole loop. The goal must lie in the lanelet **before** the
start's: with both in one lanelet Autoware routes the few decimetres behind the car and
never leaves. Measured in the RC simulator (`lab3_sim_lap`):

| loop | start past the boundary | goal behind the start | goal before the boundary | result |
|---|---|---|---|---|
| 37 m, GLIM map of the TA corridor | 0.2 m | 0.7 m | 0.5 m | lap, 36.1 m in 48 s |
| 37 m, same | 0.2 m | 1.0 m | 0.8 m | lap, 35.8 m in 48 s |
| 98 m example loop | 0.1 m | 0.7 m | 0.6 m | lap, 96.5 m |
| 98 m | 0.1 m | 0.8 m | 0.7 m | lap |
| 98 m | 0.5 m | 0.8 m | 0.3 m | lap |
| 98 m | 0.1 / 0.2 / 0.5 m | 1.0 m | 0.9 / 0.8 / 0.5 m | lap (each) |
| 98 m | 0.2 m | 0.7 m | 0.5 m | **did not move**, twice: the trajectory stops 0.4 m ahead |
| 98 m (planning_rc README) | 0.3 m | 0.5 m | 0.2 m | did not move |
| 21 m test fixture (4 x 8 m) | 0.1-0.5 m | 0.5-1.0 m | | did not start (4 pairs) |

So the rule is not a single distance: 0.7-1.0 m behind the start, with the boundary
between them, drove in 9 of 10 cases on real-sized loops. If the car does not move after
**Auto**, set the goal again about 0.3 m further back. The 21 m fixture is smaller than any
classroom loop; why it fails is not known.

`lab3_check_lanelet`'s hint says where the boundary lies relative to the origin; a boundary
just behind the start mark leaves the most room for the goal.

## Verified

Headless on a workstation (RTX GPU), ROS_DOMAIN_ID 87, 2026-11-16/17:

- **GLIM**: the exact `glim_rosbag` command with `config/glim/robin_w_ready` (log_dir pointed into
  the repository's tmp, the only change) on the TA simulation's 196 s Robin-W mapping bag:
  exit 0 in 155 s, 11 submaps dumped. The first IMU pose is (-0.005, -0.007, -0.006) with
  yaw 180 deg: the origin and axes stated above.
- **offline_viewer** on a private Xvnc display (`vglrun -d egl`), driven with xdotool
  through the clicks above: Open New Map, No, Export Points. The export is binary PLY,
  `x y z intensity` float, 1 568 229 points, 25 MB (the header `glk::save_ply_binary`
  writes). Screenshots: `tmp/lab3_glim_viewer_*.png` in the AutoSDV checkout.
- **PCL**: the three commands above on that PLY: 1 568 229 -> 48 688 points,
  `pointcloud_map.pcd` 0.78 MB, `FIELDS x y z intensity`, `DATA binary`.
- **Autoware loads it**: the map loader in the planning simulator and in the replay read
  that file (the TA corridor's lanelets moved into GLIM's frame beside it).
- **lab3_check_lanelet** on that folder: loop 4 lanelets 37.2 m, clockwise
  (-97.8 m^2), speed limits 5 km/h; hint: origin 3.02 m past lanelet 324's start, so a
  0.7 m goal falls in the start's own lanelet. The simulator laps in the table above.
- **lab3_planning_sim.launch.xml**: up headless (`rviz:=false`), localization
  initialized at the origin; `play_launch dump` resolves all five Lab 3 launches.
- **lab3_check_bag** on the TA mapping bag: all seven rows PASS (199 Hz IMU, 196 s,
  0.60 m/s, still 14 s / 12 s).
- **Replay**: `lab3_replay.launch.xml` with `ros2 launch`, the exact `ros2 bag play`
  command on the TA run bag (abl2_both; its stamps rewritten to the header times, see
  below), `/initialpose` as RViz sends it at 8 s while the car stood: localization
  INITIALIZED; NVTL over 86 s 1.26-1.72 (median 1.66; the indoor file's gate is 1.3, and
  NVTL scales with the 1.0 m resolution, so the outdoor ~2 does not apply); the EKF path
  against the path recorded on the run (moved into GLIM's frame): median 0.038 m, p95
  0.097 m, max 0.33 m; length 72.3 m against 72.5 m.
- `unset_env` takes DISPLAY and LD_PRELOAD out of `ros2 launch`'s child processes (tested
  with `printenv`); with `replay:=true` they stay.

## Not verified

- Nothing on a real car: the mapping launch with the ZED and the Robin-W, GLIM on the
  Orin, a real classroom bag, the run.
- The TA simulation's bags carry header stamps on the simulation's clock and bag times
  on the wall clock, so `ros2 bag play --clock` on them publishes a clock the stamps do
  not match; the replay was verified on a copy with each bag time set to its header
  stamp. A car's bag has both on the wall clock and plays as is.
- The GPU `glim_rosbag` needed `CUDA_MPS_PIPE_DIRECTORY` pointed at an empty directory
  on this workstation, where another user's CUDA MPS daemon refuses clients; a car or a
  student's computer has none.
- The RViz clicks themselves (2D Pose Estimate, 2D Goal Pose, Auto) were sent as the
  messages and service calls they produce, not clicked. The `rviz2` command was run on a
  private display: it opens `autosdv.rviz` with the AutowareStatePanel and both tools.

## The map folder and `lab3_map.yaml`

The TA tools record what they did in `lab3_map.yaml` inside the folder
(`lab3_check_lanelet`'s result and hint; `lab3_downsample`'s voxel). Students' folders may
have none; nothing requires it.

## TA tools

| recipe | what |
|---|---|
| `just lab3 stack car_b` | the mapping launch under play_launch, with the camera's display handling |
| `just lab3 record --car car_b --team 3 [--kind run --map DIR]` | `ros2 bag record` of the same topic lists, plus a metadata topic; a mapping bag is checked on Ctrl-C |
| `just lab3 map BAG` | GLIM headless with a per-run config from the bag's own TF -> `BAG_glim/raw_map.pcd` |
| `just lab3 downsample CLOUD MAP --voxel 0.1` | a raw `.pcd` or GLIM's exported `.ply` -> `MAP/pointcloud_map.pcd` (+ `map_projector_info.yaml`) |
| `just lab3 check-lanelet MAP`, `sim-lap MAP` | the loop check; the simulator lap alone |
| `just lab3 planning-sim MAP`, `run car_b MAP`, `replay car_b MAP` | the launches above under play_launch |
| `just lab3 lanelet-speed MAP --corners 4` | edit the lanelets' speed tags in place |
| `just lab3 grade BAG` | grade a recorded lap again |

`lab3_route` (scripted waypoint legs, the earlier procedure) is still installed for the
TA; no student step and no recorded topic uses it.

## Known limits

- **One recorder at a time in week 2.** On Humble every `ros2 bag record` is the node
  `/rosbag2_recorder`; with two running, Autoware's duplicated-node check reports ERROR
  and autonomous mode never becomes available. During the graded lap the grader is the
  recorder.
- **The field stack needs perception.** Behaviour planning waits for the occupancy grid
  and emergency braking for the obstacle cloud, so `launch_perception:=false` drives
  nowhere. Perception's compare-map filter loads the map whole
  (`config/pointcloud_map_filter_indoor.param.yaml`): its dynamic loader, built for city
  maps, crashed the pointcloud container on a Lab 3 map.
- **Indoor NDT is provisional** (`config/ndt_indoor*.param.yaml`): resolution 1.0, the
  NVTL gate 1.3 (it scales with the voxel size), `required_distance` 3.0. Built-in NDT
  only (`pose_source:=ndt`, the default): the CUDA matcher reads its own file.
- **GLIM publishes TF** (`map -> odom -> zedxm_imu_link`, `zedxm_imu_link -> robin_w`) while
  it runs; with the mapping launch still up the frames would have two parents. Stop the
  launch first.
- **The Robin-W's position is a placeholder** in every car's calibration (only its
  rotation is right; the camera is about 13 cm below it on car B), so the ready GLIM
  configs' `T_lidar_imu` lever arm will change once the mount is measured: edit `robin_w`
  in the car's `sensor_kit_calibration.yaml` and regenerate
  (`config/glim/README.md`). Car C's camera position is the kit's placeholder too, and
  its ZED's factory IMU offset was not read, so its config differs from car B's.
- **This workstation's DDS.** `cyclonedds.xml` asks for a 10 MB socket buffer; with
  `net.core.rmem_max` at 1 MB no node starts. Raise it (`sudo sysctl -w
  net.core.rmem_max=10485760`) or point `CYCLONEDDS_URI` at a copy with `min="default"`.

## TA rehearsal

The course repository's simulation (`2026/labs/Lab3/ta/sim/`) still drives the earlier
board-anchored sequence (`lab3_anchor`, the board initializer, `lab3_route`); it needs
updating to this procedure before its next run.
