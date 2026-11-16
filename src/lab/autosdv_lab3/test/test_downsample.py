"""The downsampler, and the PLY it reads as GLIM's offline_viewer exports it."""

import numpy as np
import pytest
import yaml

from autosdv_lab3 import downsample, mapdir
from autosdv_lab3 import pointcloud_io as pc


def write_glim_ply(path, points, intensity):
    """A PLY with the header glk::save_ply_binary writes for vertices + intensities
    (GLIM's File > Export Points): binary_little_endian, float x y z intensity."""
    n = len(points)
    header = ('ply\nformat binary_little_endian 1.0\ncomment generated with iridescence\n'
              f'element vertex {n}\nproperty float x\nproperty float y\nproperty float z\n'
              'property float intensity\nend_header\n')
    body = np.column_stack([points, intensity]).astype('<f4').tobytes()
    with open(path, 'wb') as f:
        f.write(header.encode('ascii') + body)


def room(n=60000, seed=0):
    rng = np.random.default_rng(seed)
    p = rng.uniform([-2, -3, 0], [6, 3, 2.5], (n, 3))
    side = rng.integers(0, 3, n)
    p[side == 0, 0] = -2.0
    p[side == 1, 1] = 3.0
    p[side == 2, 2] = 0.0
    return p, rng.uniform(0, 100, n)


def test_voxel_downsample_means():
    pts = np.array([[0.01, 0.01, 0.01], [0.03, 0.03, 0.03], [0.51, 0.0, 0.0]])
    out, inten = downsample.voxel_downsample(pts, np.array([10.0, 30.0, 7.0]), 0.1)
    order = np.argsort(out[:, 0])
    assert np.allclose(out[order], [[0.02, 0.02, 0.02], [0.51, 0.0, 0.0]])
    assert np.allclose(inten[order], [20.0, 7.0])


def test_voxel_downsample_fixed_grid_is_idempotent():
    rng = np.random.default_rng(0)
    pts = rng.uniform(0, 2, (5000, 3))
    once, i1 = downsample.voxel_downsample(pts, np.ones(len(pts)), 0.2)
    twice, _ = downsample.voxel_downsample(once, i1, 0.2)
    assert len(twice) == len(once)
    with pytest.raises(ValueError):
        downsample.voxel_downsample(pts, None, 0.0)


def test_reads_glim_export_ply(tmp_path):
    p, i = room(1000)
    write_glim_ply(tmp_path / 'map.ply', p, i)
    cloud = pc.read_cloud(str(tmp_path / 'map.ply'))
    assert len(cloud) == 1000 and cloud.has_intensity
    assert np.allclose(cloud.points, p, atol=1e-5)


@pytest.mark.parametrize('ext', ['ply', 'pcd'])
def test_downsample_into_map_folder(tmp_path, capsys, ext):
    p, i = room()
    src = tmp_path / f'raw.{ext}'
    if ext == 'ply':
        write_glim_ply(src, p, i)
    else:
        pc.write_pcd(str(src), pc.PointCloud(p, i))
    out = tmp_path / 'map'
    assert downsample.main([str(src), str(out), '--voxel', '0.2', '--max-mb', '50']) == 0
    assert 'PASS' in capsys.readouterr().out
    small = pc.read_cloud(str(out / mapdir.PCD))
    assert small.has_intensity and len(small) < len(p) / 2
    assert (out / mapdir.PROJECTOR).read_text().strip() == 'projector_type: Local'
    meta = mapdir.load_meta(str(out))
    assert meta['downsample']['voxel_m'] == 0.2
    # a tiny limit fails, and says so
    assert downsample.main([str(src), str(out), '--voxel', '0.2', '--max-mb', '0.001']) == 1
    with open(out / mapdir.MAP_META) as f:
        assert yaml.safe_load(f)['downsample']['max_mb'] == 0.001


def test_downsample_needs_a_cloud(tmp_path):
    assert downsample.main([str(tmp_path / 'none.pcd'), str(tmp_path / 'map'), '--voxel', '0.1']) == 2
