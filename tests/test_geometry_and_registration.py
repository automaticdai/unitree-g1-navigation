import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from g1_navigation.geometry import transform, apply, planar, relative_velocity, initial_alignment
from g1_navigation.registration import register, map_tree


def test_mount_rotation_and_translation_are_composed_not_renamed():
    mount = transform([0.2, 0., 1.2], Rotation.from_euler('z', 0.3).as_quat())
    base = transform([2., -1., 0.05], Rotation.from_euler('xyz', [0.1, 0.05, 1.]).as_quat())
    imu = base @ mount
    np.testing.assert_allclose(imu @ np.linalg.inv(mount), base, atol=1e-12)
    flat = planar(base)
    np.testing.assert_allclose(flat @ (np.linalg.inv(flat) @ base) @ mount, imu, atol=1e-12)
    assert flat[2, 3] == 0.


def test_initial_alignment_handles_rotated_mount():
    # G1 head Mid360: upside down (roll pi) and pitched; FAST-LIO's camera_init is the
    # IMU's own frame at start-up, so the world needs the full mount rotation.
    mount = transform([0.05, 0., 1.2], Rotation.from_euler('xyz', [np.pi, 0.4, 0.]).as_quat())
    align = initial_alignment(mount, 1.25)
    # At start-up (identity FAST-LIO pose) the base is level at the floor origin.
    base = align @ np.eye(4) @ np.linalg.inv(mount)
    np.testing.assert_allclose(base[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(base[:3, 3], [0., 0., 0.05], atol=1e-12)
    # A floor point 2 m ahead of the base, expressed in the IMU frame, lands on z = 0.
    floor_in_imu = apply(np.linalg.inv(align), [[2., 0., 0.]])
    np.testing.assert_allclose(apply(align, floor_in_imu), [[2., 0., 0.]], atol=1e-12)
    # Later FAST-LIO motion (1 m forward in the base frame) maps to world motion.
    step = np.linalg.inv(mount) @ transform([1., 0., 0.]) @ mount  # camera_init-frame motion of the IMU
    moved = align @ step @ np.linalg.inv(mount)
    np.testing.assert_allclose(moved[:3, 3], [1., 0., 0.05], atol=1e-12)
    with pytest.raises(ValueError):
        initial_alignment(mount, 0.)


def test_velocity_is_in_child_frame_and_yaw_wrap_is_short():
    previous = transform([0., 0., 0.], Rotation.from_euler('z', np.pi - 0.01).as_quat())
    current = transform([-0.01, 0., 0.], Rotation.from_euler('z', -np.pi + 0.01).as_quat())
    v, w = relative_velocity(previous, current, 0.1)
    assert v[0] == pytest.approx(0.1, abs=0.001)
    assert w[2] == pytest.approx(0.2)
    with pytest.raises(ValueError):
        relative_velocity(previous, current, 0.)


def test_invalid_pose_rejected():
    with pytest.raises(ValueError):
        transform(quaternion=[0., 0., 0., 0.])
    with pytest.raises(ValueError):
        transform(xyz=[float('nan'), 0., 0.])


def test_local_registration_recovers_known_planar_offset():
    rng = np.random.default_rng(8)
    source = rng.uniform([-3, -2, 0.2], [3, 2, 2.], (2500, 3))
    expected = transform([0.12, -0.08, 0.], Rotation.from_euler('z', 0.04).as_quat())
    target = apply(expected, source)
    actual, fitness, error = register(source, map_tree(target, voxel=0.01), np.eye(4))
    np.testing.assert_allclose(actual, expected, atol=0.005)
    assert fitness > 0.99
    assert error < 0.01


def test_registration_rejects_no_overlap_and_degenerate_points():
    rng = np.random.default_rng(1)
    points = rng.normal(size=(100, 3))
    with pytest.raises(ValueError):
        register(points + 100., map_tree(points), np.eye(4))
    line = np.column_stack((np.linspace(0, 3, 100), np.zeros(100), np.ones(100)))
    with pytest.raises(ValueError):
        register(line, map_tree(line, voxel=0.01), np.eye(4))
