"""Local 3D cloud matching constrained to x/y/yaw for a single level floor.

Requires a nearby initial pose. This is not global place recognition or loop closure.
"""
import numpy as np
from scipy.spatial import cKDTree
from .geometry import apply, planar


def register(source, target_tree, initial, max_distance=0.4, iterations=25):
    source = np.asarray(source, float).reshape(-1, 3)
    source = source[np.isfinite(source).all(axis=1)]
    if len(source) < 30:
        raise ValueError('Insufficient points for registration')
    correction = planar(initial)
    for _ in range(iterations):
        moved = apply(correction, source)
        distances, indices = target_tree.query(moved, distance_upper_bound=max_distance)
        valid = np.isfinite(distances)
        if valid.sum() < 30:
            raise ValueError('Insufficient map overlap')
        a, b = moved[valid, :2], target_tree.data[indices[valid], :2]
        ac, bc = a.mean(axis=0), b.mean(axis=0)
        if np.linalg.eigvalsh(np.cov(a.T)).min() < 0.01:
            raise ValueError('Insufficient spatial spread')
        u, _, vt = np.linalg.svd((a - ac).T @ (b - bc))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        delta = np.eye(4)
        delta[:2, :2] = rotation
        delta[:2, 3] = bc - rotation @ ac
        correction = delta @ correction
        if np.linalg.norm(delta - np.eye(4)) < 1e-5:
            break
    distances, _ = target_tree.query(apply(correction, source), distance_upper_bound=max_distance)
    valid = np.isfinite(distances)
    fitness = float(valid.mean())
    rmse = float(np.sqrt(np.mean(distances[valid] ** 2))) if valid.any() else float('inf')
    return correction, fitness, rmse


def map_tree(points, voxel=0.1):
    points = np.asarray(points, float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 30:
        raise ValueError('Map contains too few finite points')
    _, indices = np.unique(np.floor(points / voxel).astype(np.int64), axis=0, return_index=True)
    return cKDTree(points[indices])
