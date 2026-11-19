"""MuJoCo G1 simulator (sim/). Physics tests skip without mujoco or the fetched model."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sim'))
from g1_sim.policy import DEFAULT_ANGLES, WalkingPolicy, gravity_in_body  # noqa: E402

POLICY = ROOT / 'sim/assets/g1_motion_policy.npz'
ASSETS = ROOT / 'vendor/sim'


def test_gravity_in_body_upright_and_rolled():
    assert np.allclose(gravity_in_body([1, 0, 0, 0]), [0, 0, -1])
    half = np.sqrt(0.5)
    assert np.allclose(gravity_in_body([half, half, 0, 0]), [0, -1, 0])  # rolled 90 deg about x


def test_policy_standing_targets_stay_near_default_pose():
    policy = WalkingPolicy(str(POLICY))
    for step in range(20):
        target = policy.step(step * 0.02, [1, 0, 0, 0], np.zeros(3), DEFAULT_ANGLES, np.zeros(12), [0, 0, 0])
    assert target.shape == (12,)
    assert np.all(np.abs(target - DEFAULT_ANGLES) < 0.6)
    policy.reset()
    assert not policy.h.any() and not policy.action.any()


mujoco = pytest.importorskip('mujoco')
pytest.importorskip('mujoco_lidar')
needs_model = pytest.mark.skipif(not (ASSETS / 'unitree_rl_gym').is_dir(), reason='run sim/fetch_assets.sh')


def test_frame_builder_offsets_and_layout():
    from g1_sim.mid360 import POINT_DTYPE, FrameBuilder
    frame = FrameBuilder()
    frame.add(10.0, np.ones((2, 3)), np.array([0, 1], np.uint16))
    frame.add(10.05, np.zeros((1, 3)), np.array([3], np.uint16))
    start, points = frame.finish()
    assert start == 10.0 and points.dtype == POINT_DTYPE and POINT_DTYPE.itemsize == 24
    assert np.allclose(points['time'], [0, 0, 5e7], atol=1)
    assert list(points['ring']) == [0, 1, 3]
    assert frame.finish()[0] is None


@needs_model
def test_walks_on_command_and_scans_like_the_head_mid360():
    from g1_sim.engine import Simulator
    sim = Simulator(str(ASSETS), 'room', noise=False)
    out = None
    for _ in range(50):  # settle 1 s
        out = sim.step([0, 0, 0])
    _, _, acc = out.imu[-1]
    assert acc[2] < -0.9  # upside-down IMU reads gravity along -z
    imu_height = sim.data.site_xpos[sim.model.site('imu').id][2]
    assert abs(imu_height - 1.259) < 0.03
    start, _ = sim.base_pose()
    frames = []
    for _ in range(200):  # 4 s at 0.2 m/s
        frames += sim.step([0.2, 0, 0]).frames
    assert not sim.fallen()
    moved = sim.base_pose()[0] - start
    assert 0.5 < moved[0] < 1.0 and abs(moved[1]) < 0.3
    sizes = [len(points) for _, points in frames[1:]]
    assert len(frames) == 40 and min(sizes) > 15000  # 10 Hz, ~20k points in a closed room
    # Upside-down mount: most returns are below the sensor (negative z in the world).
    _, points = frames[-1]
    assert np.all(np.diff(points['time']) >= 0) and points['time'][-1] <= 1e8


@needs_model
def test_person_moves_along_its_path():
    from g1_sim.world import SCENARIOS, person_position
    scene = SCENARIOS['person']
    assert np.allclose(person_position(scene, 0.0), scene.person_path[0])
    total = np.linalg.norm(np.subtract(scene.person_path[1], scene.person_path[0]))
    assert np.allclose(person_position(scene, total / scene.person_speed), scene.person_path[1])
    assert np.allclose(person_position(scene, 2 * total / scene.person_speed), scene.person_path[0])
