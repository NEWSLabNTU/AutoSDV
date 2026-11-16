"""`lab3_downsample` (TA tool): a raw cloud down to the map folder's pointcloud_map.pcd.

    lab3_downsample CLOUD MAP_DIR --voxel V [--max-mb 50]

Reads CLOUD (lab3_map's raw_map.pcd, or the .ply GLIM's offline_viewer exports), keeps
one point per V-metre voxel -- the mean position and the mean intensity of the points
in it -- and writes MAP_DIR/pointcloud_map.pcd, plus map_projector_info.yaml
(projector_type: Local) if the folder has none. Students do the same with stock PCL
(pcl_ply2pcd, pcl_voxel_grid, pcl_convert_pcd_ascii_binary); this is the scripted path
for the TA simulation. The choice of V is the user's: larger is smaller and faster to
load, but every voxel averages away detail NDT matches against.
"""

import argparse
import os
import sys

import numpy as np

from autosdv_lab3 import mapdir

DEFAULT_MAX_MB = 50.0    # provisional, until Vector Map Builder's upload limit is measured


def voxel_downsample(points: np.ndarray, intensity, voxel: float):
    """One point per occupied voxel: the mean of its points, and of their intensity.

    The voxel index is floor(p / voxel), so the grid is fixed in the map frame and
    the same cloud always gives the same answer.
    """
    if voxel <= 0:
        raise ValueError('voxel must be positive')
    keys = np.floor(points / voxel).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    n = len(counts)
    out = np.zeros((n, 3))
    for axis in range(3):
        out[:, axis] = np.bincount(inverse, weights=points[:, axis], minlength=n) / counts
    out_i = None
    if intensity is not None:
        out_i = np.bincount(inverse, weights=intensity, minlength=n) / counts
    return out, out_i


def main(argv=None):
    ap = argparse.ArgumentParser(prog='lab3_downsample',
                                 description='Downsample a raw map cloud into a Lab 3 map folder.')
    ap.add_argument('cloud', help='the raw cloud: .pcd (lab3_map) or .ply (GLIM export)')
    ap.add_argument('map_dir', help='the map folder to write pointcloud_map.pcd into')
    ap.add_argument('--voxel', type=float, required=True, help='voxel size, metres')
    ap.add_argument('--max-mb', type=float, default=DEFAULT_MAX_MB,
                    help=f'the size limit of pointcloud_map.pcd (default {DEFAULT_MAX_MB:g} MB)')
    args = ap.parse_args(argv)

    from autosdv_lab3.pointcloud_io import PointCloud, read_cloud, write_pcd

    if not os.path.isfile(args.cloud):
        print(f'lab3_downsample: no cloud at {args.cloud}', file=sys.stderr)
        return 2
    if args.voxel <= 0:
        print('lab3_downsample: --voxel must be positive', file=sys.stderr)
        return 2
    try:
        cloud = read_cloud(args.cloud)
    except (ValueError, KeyError) as e:
        print(f'lab3_downsample: {e}', file=sys.stderr)
        return 2
    pts, inten = voxel_downsample(np.asarray(cloud.points, float),
                                  None if cloud.intensity is None
                                  else np.asarray(cloud.intensity, float), args.voxel)
    os.makedirs(args.map_dir, exist_ok=True)
    out = os.path.join(args.map_dir, mapdir.PCD)
    write_pcd(out, PointCloud(points=pts, intensity=inten))
    projector = os.path.join(args.map_dir, mapdir.PROJECTOR)
    if not os.path.isfile(projector):
        with open(projector, 'w') as f:
            f.write('projector_type: Local\n')
    size_mb = os.path.getsize(out) / 1e6
    ok = size_mb <= args.max_mb

    print(f'{len(cloud)} -> {len(pts)} points at {args.voxel:g} m '
          f'({100.0 * len(pts) / max(len(cloud), 1):.1f} %)')
    print(f'{out}: {size_mb:.1f} MB  (limit {args.max_mb:g} MB)  '
          f'{"PASS" if ok else "FAIL: over the limit, use a larger voxel"}')

    mapdir.record_step(args.map_dir, 'downsample', {
        'source': os.path.abspath(args.cloud),
        'voxel_m': args.voxel,
        'points_in': len(cloud),
        'points_out': len(pts),
        'size_mb': round(size_mb, 2),
        'max_mb': args.max_mb,
        'pcd_sha256': mapdir.sha256(out),
    })
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
