"""Bounded voxel recording and conservative flat-floor occupancy export."""
import json
import os
from pathlib import Path
import shutil
import tempfile

import numpy as np
import yaml


class VoxelMap:
    def __init__(self, resolution=0.08, max_voxels=2_000_000):
        if not np.isfinite(resolution) or resolution <= 0 or max_voxels <= 0:
            raise ValueError('Invalid voxel bounds')
        self.resolution = resolution
        self.max_voxels = max_voxels
        self.cells = {}

    def add(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        keys = np.floor(points / self.resolution).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        new = {tuple(keys[i]): points[i] for i in indices}
        if len(self.cells) + sum(key not in self.cells for key in new) > self.max_voxels:
            raise ValueError('Map voxel limit reached; save and start another session')
        self.cells.update(new)

    def points(self):
        return np.asarray(list(self.cells.values()), dtype=float).reshape(-1, 3)


def occupancy(points, resolution=0.1, ground_tolerance=0.08,
              obstacle_min=0.10, obstacle_max=1.9, max_cells=16_000_000):
    points = np.asarray(points, float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points) or resolution <= 0 or not np.isfinite(resolution):
        raise ValueError('A finite nonempty map and positive resolution are required')
    ground = np.abs(points[:, 2]) <= ground_tolerance
    obstacle = (points[:, 2] >= obstacle_min) & (points[:, 2] <= obstacle_max)
    selected = points[ground | obstacle]
    if not len(selected):
        raise ValueError('No floor/obstacle observations in configured height bands')
    low = np.floor(selected[:, :2].min(axis=0) / resolution).astype(int) - 1
    high = np.floor(selected[:, :2].max(axis=0) / resolution).astype(int) + 1
    width, height = (high - low + 1).tolist()
    if width * height > max_cells:
        raise ValueError('Occupancy grid exceeds allocation limit')
    grid = np.full((height, width), 205, dtype=np.uint8)
    for mask, value in ((ground, 254), (obstacle, 0)):
        xy = np.floor(points[mask, :2] / resolution).astype(int) - low
        grid[xy[:, 1], xy[:, 0]] = value
    # No evidence is unknown, not free. Obstacles override ground observations.
    return grid, [float(low[0] * resolution), float(low[1] * resolution), 0.0]


def save_bundle(destination, points, metadata, resolution=0.1):
    destination = Path(destination).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise FileExistsError(f'Refusing to replace {destination}')
    grid, origin = occupancy(points, resolution=resolution)
    temp = Path(tempfile.mkdtemp(prefix='.map-', dir=destination.parent))
    try:
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        np.save(temp / 'points.npy', points, allow_pickle=False)
        with (temp / 'map.pcd').open('wb') as file:
            file.write(('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                        f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                        f'POINTS {len(points)}\nDATA binary\n').encode())
            file.write(points.astype('<f4').tobytes())
        with (temp / 'map.pgm').open('wb') as file:
            file.write(f'P5\n{grid.shape[1]} {grid.shape[0]}\n255\n'.encode())
            file.write(np.flipud(grid).tobytes())
        (temp / 'map.yaml').write_text(yaml.safe_dump(dict(
            image='map.pgm', mode='trinary', resolution=resolution, origin=origin,
            negate=0, occupied_thresh=0.65, free_thresh=0.196)))
        (temp / 'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False))
        # Destination must be new. Rename makes the complete bundle visible at once.
        os.rename(temp, destination)
    except BaseException:
        shutil.rmtree(temp, ignore_errors=True)
        raise
    return destination
