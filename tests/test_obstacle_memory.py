import numpy as np
import pytest

from g1_navigation.obstacle_memory import ObstacleMemory

SENSOR = np.array([0., 0., 1.26])


def box(x, y=0., height=0.35, size=0.2):
    g = np.arange(0., size + 1e-9, 0.05)
    xx, yy, zz = np.meshgrid(x + g, y - size / 2 + g, np.arange(0.1, height + 1e-9, 0.05))
    return np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])


def wall(x, y_half=1.5, top=1.9):
    yy, zz = np.meshgrid(np.arange(-y_half, y_half + 1e-9, 0.02), np.arange(0.1, top, 0.02))
    return np.column_stack([np.full(yy.size, x), yy.ravel(), zz.ravel()])


def floor(x0, x1, y_half=1.5):
    xx, yy = np.meshgrid(np.arange(x0, x1, 0.02), np.arange(-y_half, y_half + 1e-9, 0.02))
    return np.column_stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)])


def near(points, target, tolerance=0.08):
    return len(points) and np.min(np.linalg.norm(points - target, axis=1)) < tolerance


def test_unobservable_obstacle_is_kept_as_robot_approaches():
    memory = ObstacleMemory()
    seen = box(1.5)
    memory.update(SENSOR, seen, seen, (0., 0.), 0.)
    # Robot walks up; the box is now in the blind zone and the scan only shows a far wall.
    sensor = SENSOR + [1.0, 0., 0.]
    far = wall(4.)
    kept = memory.update(sensor, far, far, (1.0, 0.), 5.)
    assert near(kept, [1.5, 0., 0.3])


def test_beam_through_a_remembered_point_clears_it():
    memory = ObstacleMemory()
    person = box(1.0, height=1.6)
    memory.update(SENSOR, person, person, (0., 0.), 0.)
    # Person walked away: the same directions now return from a wall behind.
    behind = wall(3., top=3.)
    scan = np.vstack([floor(1.0, 3.), behind])
    kept = memory.update(SENSOR, scan, behind, (0., 0.), 0.1)
    old = kept[(kept[:, 0] < 1.5)]
    # Including the low points, whose beams end on the floor just behind them.
    assert len(old) == 0
    assert near(kept, [3., 0., 1.0])


def test_shorter_return_does_not_clear():
    memory = ObstacleMemory()
    far = box(2.0, height=1.6)
    memory.update(SENSOR, far, far, (0., 0.), 0.)
    # Something in front hides it: beams end before reaching it.
    front = box(1.0, height=1.6)
    kept = memory.update(SENSOR, front, front, (0., 0.), 0.1)
    assert near(kept, [2.1, 0., 1.0]) and near(kept, [1.1, 0., 1.0])


def test_points_expire_and_leave_the_radius():
    memory = ObstacleMemory(max_age=2., radius=3.)
    memory.update(SENSOR, box(1.5), box(1.5), (0., 0.), 0.)
    assert near(memory.update(SENSOR, np.empty((0, 3)), np.empty((0, 3)), (0., 0.), 1.9), [1.5, 0., 0.3])
    assert len(memory.update(SENSOR, np.empty((0, 3)), np.empty((0, 3)), (0., 0.), 2.1)) == 0
    memory.update(SENSOR, box(1.5), box(1.5), (0., 0.), 3.)
    assert len(memory.update(SENSOR + [-2.5, 0, 0], np.empty((0, 3)), np.empty((0, 3)), (-2.5, 0.), 3.1)) == 0


def test_reobservation_refreshes_and_voxels_deduplicate():
    memory = ObstacleMemory(max_age=2.)
    obstacle = box(1.5)
    for t in range(5):
        memory.update(SENSOR, obstacle, obstacle, (0., 0.), float(t))
    assert len(memory.points) <= len(obstacle) and np.all(memory.seen == 4.)
    jitter = obstacle + 0.004
    memory.update(SENSOR, jitter, jitter, (0., 0.), 5.)
    assert len(memory.points) <= len(obstacle)


def test_max_points_keeps_newest():
    memory = ObstacleMemory(max_points=10)
    memory.update(SENSOR, box(1.5), box(1.5), (0., 0.), 0.)
    memory.update(SENSOR + [0, 0, 0], np.empty((0, 3)), box(-1.5), (0., 0.), 1.)
    assert len(memory.points) == 10 and np.all(memory.seen == 1.)


def test_invalid_configuration_and_reset():
    with pytest.raises(ValueError):
        ObstacleMemory(voxel=0.)
    with pytest.raises(ValueError):
        ObstacleMemory(max_age=np.nan)
    memory = ObstacleMemory()
    memory.update(SENSOR, box(1.5), box(1.5), (0., 0.), 0.)
    memory.reset()
    assert len(memory.points) == 0


def test_points_above_the_sensor_expire_after_above_sensor_age():
    memory = ObstacleMemory(max_age=30., above_sensor_age=1.0)
    head = np.array([[1.5, 0., 1.6], [1.5, 0.1, 1.65]])   # above the 1.26 m sensor
    low = box(1.5, y=1.0)                                   # below it, in the blind zone later
    memory.update(SENSOR, np.vstack([head, low]), np.vstack([head, low]), (0., 0.), 0.)
    # Nothing re-observes either (out of view): the low box stays, the head goes.
    kept = memory.update(SENSOR, wall(4.), np.empty((0, 3)), (0., 0.), 0.9)
    assert near(kept, head[0]) and near(kept, [1.6, 1.0, 0.3])
    kept = memory.update(SENSOR, wall(4.), np.empty((0, 3)), (0., 0.), 1.5)
    assert not near(kept, head[0]) and near(kept, [1.6, 1.0, 0.3])


def test_above_sensor_points_still_observed_are_kept():
    memory = ObstacleMemory(max_age=30., above_sensor_age=1.0)
    shelf = np.array([[2.0, 0., 1.5]])
    for t in np.arange(0., 5., 0.5):  # re-observed every 0.5 s
        kept = memory.update(SENSOR, shelf, shelf, (0., 0.), t)
    assert near(kept, shelf[0])


def test_above_sensor_age_defaults_to_max_age():
    memory = ObstacleMemory(max_age=2.)
    head = np.array([[1.5, 0., 1.6]])
    memory.update(SENSOR, head, head, (0., 0.), 0.)
    assert near(memory.update(SENSOR, wall(4.), np.empty((0, 3)), (0., 0.), 1.9), head[0])
    with pytest.raises(ValueError):
        ObstacleMemory(above_sensor_age=0.)
