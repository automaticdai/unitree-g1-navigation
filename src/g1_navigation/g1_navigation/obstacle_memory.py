"""Remember obstacle points until a LiDAR beam proves the space is free.

The G1's head Mid360 is mounted upside down 1.26 m up: it cannot see the floor
within ~1 m of the feet, so a low obstacle drops out of view as the robot walks
up to it. Its non-repetitive scan pattern also misses small objects in many
individual scans. A collision monitor fed with single scans would then see
"no obstacle" exactly when one is close.

Points are kept in the odom frame. A remembered point is forgotten only when
a beam in the current scan travels through it (a return in the same direction,
farther away) - evidence that the space is now free. Points that cannot be
seen through (blind zone, gaps in the scan pattern) are kept, bounded by
max_age, radius and max_points.

Points above the sensor are kept only `above_sensor_age` after they were last
observed. The upside-down head Mid360 sees at most ~+7 deg above horizontal, so
nothing above it can ever be seen through: a passing person's head, hit only
during the robot's pitch sway, would otherwise block driving for max_age. What is
still in view keeps being re-observed and stays. The blind zone the memory exists
for (near the feet) is below the sensor and keeps max_age.
"""
import numpy as np


class ObstacleMemory:
    def __init__(self, voxel=0.05, max_age=30.0, radius=3.0, clear_margin=0.10,
                 angle_resolution=np.radians(1.0), max_points=50000, above_sensor_age=None):
        above_sensor_age = max_age if above_sensor_age is None else above_sensor_age
        values = np.array([voxel, max_age, radius, clear_margin, angle_resolution, max_points,
                           above_sensor_age], float)
        if not np.isfinite(values).all() or np.any(values <= 0) or angle_resolution > np.radians(10.):
            raise ValueError('Invalid obstacle memory configuration')
        self.voxel, self.max_age, self.radius = voxel, max_age, radius
        self.above_sensor_age = min(above_sensor_age, max_age)
        self.clear_margin, self.resolution, self.max_points = clear_margin, angle_resolution, int(max_points)
        self.reset()

    def reset(self):
        self.points = np.empty((0, 3))
        self.seen = np.empty(0)

    def _bins(self, vectors):
        horizontal = np.hypot(vectors[:, 0], vectors[:, 1])
        azimuth = np.floor((np.arctan2(vectors[:, 1], vectors[:, 0]) + np.pi) / self.resolution).astype(np.int64)
        elevation = np.floor((np.arctan2(vectors[:, 2], horizontal) + np.pi / 2) / self.resolution).astype(np.int64)
        return elevation * (int(2 * np.pi / self.resolution) + 2) + azimuth

    def update(self, sensor, scan, obstacles, robot, now):
        """Add this scan's obstacles; return all remembered obstacle points (odom frame).

        sensor: (3,) sensor origin; scan: all returns (beam ends) after self
        filtering; obstacles: the subset to remember; robot: (2,) robot xy.
        """
        sensor, robot = np.asarray(sensor, float), np.asarray(robot, float)
        scan, obstacles = np.asarray(scan, float).reshape(-1, 3), np.asarray(obstacles, float).reshape(-1, 3)
        points, seen = self.points, self.seen
        if len(points) and len(scan):
            # Farthest return per direction bin in this scan.
            rays = scan - sensor
            bins, ranges = self._bins(rays), np.linalg.norm(rays, axis=1)
            order = np.argsort(bins)
            bins, ranges = bins[order], ranges[order]
            unique, start = np.unique(bins, return_index=True)
            farthest = np.maximum.reduceat(ranges, start)
            offsets = points - sensor
            query, distance = self._bins(offsets), np.linalg.norm(offsets, axis=1)
            index = np.clip(np.searchsorted(unique, query), 0, len(unique) - 1)
            seen_through = (unique[index] == query) & (farthest[index] > distance + self.clear_margin)
            points, seen = points[~seen_through], seen[~seen_through]
        points = np.vstack([points, obstacles])
        seen = np.r_[seen, np.full(len(obstacles), float(now))]
        age = now - seen
        keep = ((age <= self.max_age) & (np.hypot(*(points[:, :2] - robot).T) <= self.radius)
                & ((points[:, 2] <= sensor[2]) | (age <= self.above_sensor_age)))
        points, seen = points[keep], seen[keep]
        if len(points):
            # One point per voxel, newest observation wins.
            keys = np.floor(points / self.voxel).astype(np.int64)
            order = np.argsort(-seen, kind='stable')
            _, first = np.unique(keys[order], axis=0, return_index=True)
            chosen = order[first]
            chosen = chosen[np.argsort(-seen[chosen], kind='stable')][:self.max_points]
            points, seen = points[chosen], seen[chosen]
        self.points, self.seen = points, seen
        return points
