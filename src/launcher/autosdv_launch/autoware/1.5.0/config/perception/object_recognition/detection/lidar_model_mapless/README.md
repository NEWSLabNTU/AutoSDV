# Mapless lidar model params

Copies of `autoware_launch`'s lidar_model files, differing only in
`densification_params.world_frame_id: map -> base_link`, selected by
`use_mapless_mode:=true`. In base_link, densification loses its map-frame TF
requirement, so CenterPoint runs with localization off. Correct for a
stationary vehicle; a moving vehicle gets no ego-motion compensation across
the densified frames.

`centerpoint_common.param.yaml` is an unmodified copy: the node loads it from
the same directory as the model file, so the directory must be complete.
Only centerpoint variants are provided; add `<model>_common` + `<model>` pairs
for others as needed.
