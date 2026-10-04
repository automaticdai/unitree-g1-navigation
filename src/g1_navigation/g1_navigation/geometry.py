import numpy as np
from scipy.spatial.transform import Rotation


def transform(xyz=(0., 0., 0.), quaternion=(0., 0., 0., 1.)):
    xyz, quaternion = np.asarray(xyz, float), np.asarray(quaternion, float)
    if xyz.shape != (3,) or quaternion.shape != (4,) or not np.isfinite(np.r_[xyz, quaternion]).all():
        raise ValueError('Invalid pose')
    if abs(np.linalg.norm(quaternion) - 1.) > 0.01:
        raise ValueError('Quaternion is not normalized')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = xyz
    return result


def apply(matrix, points):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def planar(matrix):
    yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
    return transform((matrix[0, 3], matrix[1, 3], 0.), Rotation.from_euler('z', yaw).as_quat())


def relative_velocity(previous, current, dt):
    if not 0.001 <= dt <= 0.5:
        raise ValueError('Odometry interval out of bounds')
    velocity = current[:3, :3].T @ ((current[:3, 3] - previous[:3, 3]) / dt)
    angular = Rotation.from_matrix(previous[:3, :3].T @ current[:3, :3]).as_rotvec() / dt
    return velocity, angular
