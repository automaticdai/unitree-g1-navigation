import json
import numpy as np
import pytest
import yaml
from g1_navigation.map_io import VoxelMap, occupancy, save_bundle


def test_voxel_limit_does_not_partially_update_map():
    voxels = VoxelMap(0.1, max_voxels=2)
    voxels.add([[0., 0., 0.], [0.01, 0., 0.], [float('nan'), 0., 0.]])
    assert len(voxels.points()) == 1
    with pytest.raises(ValueError):
        voxels.add([[1., 0., 0.], [2., 0., 0.]])
    assert len(voxels.points()) == 1


def test_obstacle_overrides_floor_and_unseen_stays_unknown():
    grid, origin = occupancy([[0., 0., 0.], [0., 0., 0.5], [0.2, 0., 0.]])
    assert origin == [-0.1, -0.1, 0.]
    assert grid[1, 1] == 0
    assert grid[1, 3] == 254
    assert grid[1, 2] == 205


def test_bundle_is_complete_and_cannot_overwrite(tmp_path):
    points = np.array([[0., 0., 0.], [0.1, 0.2, 0.5]])
    target = save_bundle(tmp_path / 'room', points, {'loop_closed': False})
    np.testing.assert_allclose(np.load(target / 'points.npy'), points)
    assert (target / 'map.pcd').read_bytes().endswith(points.astype('<f4').tobytes())
    params = yaml.safe_load((target / 'map.yaml').read_text())
    assert params['image'] == 'map.pgm'
    grid, _ = occupancy(points)
    assert (target / 'map.pgm').read_bytes().endswith(np.flipud(grid).tobytes())
    assert not json.loads((target / 'metadata.json').read_text())['loop_closed']
    with pytest.raises(FileExistsError):
        save_bundle(target, points, {})


def test_huge_grid_and_empty_maps_rejected():
    with pytest.raises(ValueError):
        occupancy([[0., 0., 0.], [1e6, 1e6, 0.]])
    with pytest.raises(ValueError):
        occupancy([])
