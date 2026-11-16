# Robin-W point cloud format, as car B publishes it

What a simulated Robin-W must publish so the rest of the stack cannot tell it
from car B's. Measured live on car B on 2026-11-16 (AutoSDV `ac6a479`,
`seyond_ros_driver` built with `ENABLE_XYZIRCAEDT`), and checked against the
driver's writer, `seyond_lidar_ros/src/driver/ros2_driver_adapter.hpp`
(`publishFrame`).

## Topics

| topic | published by | frame | rate |
|---|---|---|---|
| `/sensing/lidar/iv_points` | `seyond_ros_driver` | `robin_w` | 10 Hz |
| `/sensing/lidar/concatenated/pointcloud` | the sensor kit's passthrough (`pointcloud_preprocessor.launch.py`) | `base_link` | 10 Hz |

Both use the same point layout. The driver publishes with
`rclcpp::SensorDataQoS()` (best effort, keep last 5). The concatenated cloud is
the raw one transformed into `base_link`, unfiltered on the default CPU backend.
NDT, perception and GLIM (`just bag map`, `just lab3 map`) all read it.

## Message

- `height` 1, `width` = the point count (108,700 to 109,300 per frame
  indoors; about 95,000 in an open area)
- `is_dense` true: no NaN points, and no zero-range points
- `is_bigendian` false
- `point_step` 32, `row_step` = 32 x `width`
- `header.stamp`: the frame start, on the LiDAR's own clock. The Orin serves
  that clock over PTP (`ptp4l` + `phc2sys`). With `phc2sys` down, every stamp
  is 37 s in the past (TAI vs UTC; `docs/guides/bag-to-pointcloud-map.md`).
  A simulator stamps with sim time.

## Point layout (Autoware's `PointXYZIRCAEDT`)

| field | offset | type | observed values |
|---|---|---|---|
| `x`, `y`, `z` | 0, 4, 8 | FLOAT32 | metres, in the message's frame |
| `intensity` | 12 | UINT8 | 0..255 |
| `return_type` | 13 | UINT8 | always 1 (single return) |
| `channel` | 14 | UINT16 | 43..191 |
| `azimuth` | 16 | FLOAT32 | rad, -pi..pi |
| `elevation` | 20 | FLOAT32 | rad, 0.52..1.57 |
| `distance` | 24 | FLOAT32 | m, in 12.5 mm steps; equals `|xyz|` to within 0.07 m |
| `time_stamp` | 28 | **UINT32** | ns after `header.stamp`, 0 to ~92 ms |

`azimuth` and `elevation` are the driver's own polar angles, measured in the
sensor's native axes. They are not angles in `robin_w` or `base_link`;
`elevation` 0.52..1.57 rad is not an up/down angle. Nothing in the stack reads
these two fields, so a simulator that fills them from its own beam geometry is
safe.

`time_stamp` is what deskewing (`pointcloud_backend:=cuda`) and GLIM's
per-point timing use. A cloud without it, or with it as another type, is
refused by the CUDA preprocessor. It is an offset from the frame start, not an
absolute time.

**Order.** In `iv_points` the points are not in time order (`time_stamp`
is not monotonic). The concatenated cloud was in time order in the frames
measured. Neither consumer seen so far depends on the order.

## Frames

`/tf_static` as published on car B:

```
base_link -> sensor_kit_base_link   xyz 0 0 0, identity
sensor_kit_base_link -> robin_w     xyz 0 0 0, q (x, y, z, w) = (0.7071, 0, 0.7071, 0)
```

The Robin-W's native axes are X up, Y right, Z forward. The rotation turns them
into ROS axes (X forward, Y left, Z up). The `robin_w` translation is a
placeholder; car B's measured mount is not in `sensor_kit_calibration.yaml` yet.
The ZED's pose is measured (`zedxm_camera_link` at x 0.072, y -0.009,
z -0.14 from `sensor_kit_base_link`), so the camera sits 14 cm below the
LiDAR's frame origin.

## Field of view

From a recording (`rosbags/outdoor_20261106_195438`), in `base_link`, points
beyond 0.5 m, 0.5th to 99.5th percentile over 10 frames:

| | span |
|---|---|
| horizontal | -58.3 to +56.4 deg (about 115 deg) |
| vertical | -39.0 to +13.8 deg (about 53 deg); extremes -40.3 / +14.6 |

The vertical span is asymmetric because of how the sensor is mounted: it looks
down more than up.
