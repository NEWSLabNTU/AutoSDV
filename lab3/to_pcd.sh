#!/bin/bash
# Step 2: the exported lab3/map.ply to lab3/map/pointcloud_map.pcd, 0.1 m voxels.
set -e
cd "$(git rev-parse --show-toplevel)/lab3" && mkdir -p map glim
pcl_ply2pcd map.ply glim/raw.pcd
pcl_voxel_grid glim/raw.pcd glim/voxel.pcd -leaf 0.1,0.1,0.1
pcl_convert_pcd_ascii_binary glim/voxel.pcd map/pointcloud_map.pcd 1
echo "projector_type: Local" > map/map_projector_info.yaml
