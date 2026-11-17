# Phase 3: perception and models

- Models from `autoware-download-models-2-1-0`; CenterPoint tiny moves to
  `lidar_centerpoint/tiny/pts_backbone_neck_head.onnx`. `just setup-autoware-data` and
  `just build-engines` per version; TensorRT ABI checked as for 1.5.0.
- Presets: `lidar_only` adapted (the tracker reads `lidar_clustering` as its own channel,
  the only source of unknown objects). `camera_lidar_fusion`, detection-by-tracker and the
  image-segmentation filter have no 2.1.0 pipeline: refuse with a named error, or port
  `component_perception` and `object_recognition_lidar` from 2.1.0.
- ZED object detection and traffic lights: check each switch the presets set.

**Exit:** each perception preset runs on 2.1.0 or refuses with a named error; the engines
build once and are reused.

## Result (2026-11-18)

Done on the workstation (RTX 3090, amd64). The Orin half is phase 5.

**Overrides.** 2.1.0's `component_perception` is included as it ships, and our three
tunings reach it through `<let>` in the including scope, which outranks the nested
`<arg default>`:

| setting | value | how |
|---|---|---|
| `crop_box_margin_max_z` | 1.0 | `<let>` |
| ground filter | `non_ground_height_threshold` 0.10 | `autoware/2.1.0/config/.../scan_ground_filter.param.yaml`, by `<let scan_ground_filter_node_param_path>` |
| mapless CenterPoint | `world_frame_id: base_link` | `autoware/2.1.0/config/.../centerpoint_common.param.yaml`, only when `use_mapless_mode` |

Each was checked in the resolved launch model, on the node it is meant for. The 1.5.0
copies moved to `autoware/1.5.0/config/`; the overlay macro now requires only `launch/`
to have the same files in every version, since a config tree legitimately differs.

**Presets.** `lidar_only` and `minimal` run. Everything 2.1.0 has no lidar-only pipeline
for stops the launch with a named reason (`launch/util/refuse.launch.py`):
`perception_mode` other than `lidar`, `use_traffic_light_recognition`,
`use_detection_by_tracker`, `use_image_segmentation_based_filter`,
`use_multi_channel_tracker_merger`. So `camera_lidar_fusion` refuses. Porting it is not
part of this campaign; nothing on the cars uses it today. `enable_zed_object_detection`
is the ZED node's own detector, in the sensor kit, and does not depend on the Autoware
version.

**Model.** `lidar_detection_model` defaults to `centerpoint/centerpoint_tiny`, the
variant 1.5.0 actually ran; 2.1.0 keeps each variant in its own folder
(`lidar_centerpoint/tiny/`).

**Engines.** `just build-engines` builds the two tiny CenterPoint engines on 2.1.0
(about 3 min) and a second run reuses them (15 s). Reuse needed the TensorRT ABI:
2.1.0's amd64 Debians were also built against **10.8.0** ("Plan was created with a
different version of TensorRT! Current version: 10.8.0" while the host loaded 10.9.0),
now `autoware.versions."2.1.0".tensorrt_engine_abi`.

**Simulators.** 2.1.0 moved dummy perception and the simulated perception modules out
of `tier4_simulator_component` into `planning_simulator/perception_simulation.launch.xml`.
Without it nothing publishes `/perception/object_recognition/objects`, the behavior path
planner waits for it forever, and the RC and coach simulators never become autonomous.
`launch/components/autosdv_simulator_component.launch.xml` (per version, same arguments)
includes both on 2.1.0 and the stock component on 1.5.0.

**Exit met** on amd64: each preset runs or refuses by name; the engines build once and
are reused.
