# autosdv_cuda_preprocessor

## Purpose

Autoware's `autoware_cuda_pointcloud_preprocessor` accelerates most of the CPU
preprocessing chain, but two stages the localization chain needs have no CUDA
counterpart there:

- **crop box** — CUDA cropping exists only inside `cuda_pointcloud_preprocessor`,
  fused with distortion correction and ring outlier filtering as one sensing-side
  node. There is no standalone CUDA crop box to drop in where
  `autoware::pointcloud_preprocessor::CropBoxFilterComponent` sits.
- **random downsample** — no CUDA version at all.

The localization preprocessing chain is crop box → voxel grid downsample →
random downsample. With those two missing, a `cuda_ndt_matcher` deployment pays a
device-to-host copy before the crop box and a host-to-device copy after the
random downsample, sandwiching the one accelerated stage between two transfers.
This package supplies the missing ends so the chain stays GPU-resident from the
concatenated cloud to NDT's input.

Both nodes read and write `cuda_blackboard::CudaPointCloud2` and chain directly
with `autoware::cuda_pointcloud_preprocessor::CudaVoxelGridDownsampleFilterNode`
between them. `cuda_ndt_matcher_launch`'s `util/util.launch.xml` wires all three
under `localization_pointcloud_backend:=cuda`.

## Nodes

### `cuda_crop_box_filter_node`

Axis-aligned crop box on the GPU, mirroring
`autoware::pointcloud_preprocessor::CropBoxFilterComponent`.

| Direction | Topic                   | Type                                |
| --------- | ----------------------- | ----------------------------------- |
| Input     | `~/input/pointcloud`    | `cuda_blackboard::CudaPointCloud2`  |
| Output    | `~/output/pointcloud`   | `cuda_blackboard::CudaPointCloud2`  |

| Parameter     | Type   | Default   | Meaning                                                                 |
| ------------- | ------ | --------- | ----------------------------------------------------------------------- |
| `min_x`       | double | required  | lower x bound, inclusive [m]                                            |
| `max_x`       | double | required  | upper x bound, inclusive [m]                                            |
| `min_y`       | double | required  | lower y bound, inclusive [m]                                            |
| `max_y`       | double | required  | upper y bound, inclusive [m]                                            |
| `min_z`       | double | required  | lower z bound, inclusive [m]                                            |
| `max_z`       | double | required  | upper z bound, inclusive [m]                                            |
| `negative`    | bool   | `false`   | `false` keeps points inside the box, `true` keeps the ones outside      |
| `input_frame` | string | `""`      | the frame the bounds are expressed in; empty accepts any frame          |

The bounds are inclusive, as in the CPU component: a point exactly on a face is
inside the box.

Points with a non-finite x, y or z are dropped **in both polarities**. This is
not symmetry for its own sake — every comparison against NaN is false, so `inside`
is false for a NaN point and a naive `negative` implemented as `!inside` would
keep it and hand NDT a NaN. The CPU component does not do that, and neither does
this one.

The point layout is passed through untouched. Cropping selects whole points and
copies `point_step` bytes each, so intensity, ring, timestamp, vendor fields and
even alignment padding survive without this filter knowing they exist. Only the
x, y and z offsets are read, and they are looked up in the incoming `fields`
rather than assumed; a cloud with no float32 x/y/z is dropped with an error
rather than reinterpreted.

Configuration: [`config/cuda_crop_box_filter.param.yaml`](config/cuda_crop_box_filter.param.yaml),
which mirrors the localization chain's
`crop_box_filter_measurement_range.param.yaml` so the CUDA node is a drop-in for
the CPU one there.

#### Caveat: this node does not transform frames

The CPU component can crop in a frame other than the cloud's own, looking up tf
to get there, and it has an `output_frame` to transform the result into. This one
does neither, and deliberately has no `output_frame` parameter — offering it
would only invite someone to set it and believe it.

`input_frame` is therefore an assertion, not a request. A cloud whose
`header.frame_id` differs is **dropped with an error**, because cropping the
right box in the wrong frame removes the wrong points and nothing downstream
would report it. Leaving `input_frame` empty disables the check and accepts any
frame, which is only safe when the producer's frame is already known to match.

In the localization chain both frames are `base_link`, so nothing is lost.

### `cuda_random_downsample_filter_node`

Random downsample on the GPU, the counterpart of
`autoware::pointcloud_preprocessor::RandomDownsampleFilterComponent`: at most
`sample_num` points survive, chosen uniformly at random, and an input already at
or below the budget passes through whole. Same blackboard topics
(`~/input/pointcloud`, `~/output/pointcloud`) and the same whole-point copy, so
the layout survives here too. Configuration:
[`config/cuda_random_downsample_filter.param.yaml`](config/cuda_random_downsample_filter.param.yaml).

## Building

The package is CUDA end to end and has no CPU fallback to offer, so
`CMakeLists.txt` **skips itself** when no CUDA toolkit is found rather than
installing nodes that cannot load. The CPU localization chain is unaffected by
that skip.

`CMAKE_CUDA_ARCHITECTURES` defaults to `87;86;89` — sm_87 is the AGX Orin, the
target board; the rest cover the development desktops. Pass
`-DCMAKE_CUDA_ARCHITECTURES=87` to trim build time on the Orin.

## Testing

```bash
colcon test --packages-select autosdv_cuda_preprocessor
```

The unit tests drive the filter classes directly against real device memory —
there is no CPU reference implementation to check against, and a mocked CUDA
runtime would only test the mock. They `GTEST_SKIP` when `cudaGetDeviceCount`
reports no device, so a build host without a GPU reports skipped rather than
failed.
