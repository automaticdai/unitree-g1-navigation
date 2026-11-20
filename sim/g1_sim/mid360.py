"""Simulated Livox Mid360 (head of the G1), shaped like Unitree's /utlidar stream.

Scan pattern: Livox's recorded non-repetitive Mid360 table (azimuth, elevation),
shipped by MuJoCo-LiDAR (MIT, `pip install mujoco-lidar`), fired in table order
at 200,000 points/s. Rays are cast in chunks from the sensor pose at the time
of each chunk, so walking and turning distort a frame as on the real sensor.

Output matches what utlidar_relay receives from the robot: a PointCloud2 in the
LiDAR frame with x, y, z, intensity, per-point `time` (ns since the frame
stamp) and `ring`; IMU in its own frame with gyro in rad/s and acceleration in g.
"""
from importlib import resources

import mujoco
import numpy as np

from .world import ROBOT_GROUP

GRAVITY = 9.80665
POINT_RATE = 200_000      # points/s, first return
MIN_RANGE, MAX_RANGE = 0.1, 40.0  # datasheet: 0.1 m blind zone, 40 m at 10 % reflectivity
RANGE_SIGMA = 0.02        # m (datasheet: <= 2 cm at 10 m, 1 sigma)
ANGLE_SIGMA = np.radians(0.15)
# ICM-40609 noise densities (Mid360 datasheet / TDK): 4.5 mdps/sqrt(Hz), 100 ug/sqrt(Hz).
GYRO_DENSITY = np.radians(4.5e-3)
ACC_DENSITY = 100e-6       # g/sqrt(Hz)

CLOUD_FIELDS = [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7),
                ('time', 16, 7), ('ring', 20, 4)]  # (name, offset, PointField datatype)
POINT_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'time', 'ring'],
                        'formats': ['<f4', '<f4', '<f4', '<f4', '<f4', '<u2'],
                        'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24})


def scan_table():
    """Mid360 (azimuth, elevation) in radians, in firing order."""
    with resources.as_file(resources.files('mujoco_lidar') / 'scan_mode' / 'mid360.npy') as path:
        return np.load(path).astype(np.float64)


class Mid360:
    def __init__(self, model, rng=None, noise=True, table=None):
        self.model = model
        self.rng = rng or np.random.default_rng()
        self.noise = noise
        table = scan_table() if table is None else np.asarray(table, float)
        az, el = table[:, 0], table[:, 1]
        self.directions = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
        self.cursor = 0
        self.site = model.site('lidar').id
        self.imu_site = model.site('imu').id
        self.acc_adr = model.sensor_adr[model.sensor('imu_acc').id]
        self.gyro_adr = model.sensor_adr[model.sensor('imu_gyro').id]
        groups = np.ones(6, np.uint8)
        groups[ROBOT_GROUP] = 0
        self.groups = groups

    def next_directions(self, count):
        index = (self.cursor + np.arange(count)) % len(self.directions)
        self.cursor = int((self.cursor + count) % len(self.directions))
        return self.directions[index], index

    def cast(self, data, count):
        """Fire the next `count` rays from the current sensor pose.

        Returns (points (n,3) in the LiDAR frame, ring (n,)) for rays that returned.
        """
        local, index = self.next_directions(count)
        if self.noise:
            local = local + self.rng.normal(0, ANGLE_SIGMA, local.shape)
            local /= np.linalg.norm(local, axis=1, keepdims=True)
        rotation = data.site_xmat[self.site].reshape(3, 3)
        world = (local @ rotation.T).reshape(-1)
        dist = np.empty(count)
        geom = np.empty(count, np.int32)
        mujoco.mj_multiRay(self.model, data, data.site_xpos[self.site].copy(), world, self.groups, 1, -1,
                           geom, dist, count, MAX_RANGE)
        hit = (geom >= 0) & (dist >= MIN_RANGE)
        dist = dist[hit]
        if self.noise:
            dist = dist + self.rng.normal(0, RANGE_SIGMA, dist.shape)
        return local[hit] * dist[:, None], (index[hit] % 4).astype(np.uint16)

    def imu(self, data, rate):
        """(gyro rad/s, acceleration in g) in the IMU frame, with white noise at `rate` Hz."""
        gyro = data.sensordata[self.gyro_adr:self.gyro_adr + 3].copy()
        acc = data.sensordata[self.acc_adr:self.acc_adr + 3] / GRAVITY
        if self.noise:
            gyro += self.rng.normal(0, GYRO_DENSITY * np.sqrt(rate), 3)
            acc = acc + self.rng.normal(0, ACC_DENSITY * np.sqrt(rate), 3)
        return gyro, acc


class FrameBuilder:
    """Accumulates timed chunks into one frame with per-point time offsets."""

    def __init__(self):
        self.start = None
        self.parts = []

    def add(self, time, points, ring):
        if self.start is None:
            self.start = time
        self.parts.append((time - self.start, points, ring))

    def finish(self):
        """Return (frame start time, structured point array) and reset."""
        count = sum(len(points) for _, points, _ in self.parts)
        out = np.zeros(count, POINT_DTYPE)
        i = 0
        for offset, points, ring in self.parts:
            n = len(points)
            out['x'][i:i + n], out['y'][i:i + n], out['z'][i:i + n] = points.T
            out['intensity'][i:i + n] = 100.0
            out['time'][i:i + n] = offset * 1e9
            out['ring'][i:i + n] = ring
            i += n
        start, self.start, self.parts = self.start, None, []
        return start, out
