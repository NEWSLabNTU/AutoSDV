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
