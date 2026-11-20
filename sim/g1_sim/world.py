"""Build the MuJoCo world: Unitree's 12-DoF G1 (arms and waist fixed) in a test room.

The G1 model comes from unitree_rl_gym (`resources/robots/g1_description/g1_12dof.xml`,
BSD-3-Clause), fetched by `sim/fetch_assets.sh`. This module adds the floor, the
room, scenario obstacles, the head Mid360 IMU/LiDAR sites and IMU sensors.
"""
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

# Head Mid360 IMU in the pelvis frame. Horizontal position: Unitree's URDF (pelvis ->
# torso -0.0039635, 0, 0.054; torso -> mid360 0.0002835, 0.00003, 0.40618). Height:
# URDF 0.460 raised 1.9 cm so the IMU is 1.259 m above the floor while the policy
# walks (pelvis ~0.78 m), as measured on the real robot. Orientation: the measured
# mount (config/robot_g1_head.yaml, base_to_imu_xyzw), upside down, ~3.2 deg tilt.
IMU_POS = (-0.0036800, 0.00003, 0.479)
IMU_HEIGHT = 1.259  # measured on the real robot; engine.calibrate_mount() matches it
IMU_QUAT_XYZW = (0.9996, 0.00006, -0.00195, 0.02826)
# LiDAR origin in the IMU frame (fastlio_g1.yaml extrinsic_T; rotation identity).
LIDAR_IN_IMU = (-0.011, -0.02329, 0.04412)
ROBOT_GROUP = 4  # geom group for robot geoms, excluded from LiDAR rays
LEG_JOINTS = [f'{side}_{name}_joint' for side in ('left', 'right')
              for name in ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')]


@dataclass
class Box:
    name: str
    pos: tuple   # centre (x, y, z), world frame, metres
    size: tuple  # half extents
    rgba: tuple = (0.75, 0.45, 0.2, 1.0)


@dataclass
class Scenario:
    name: str
    boxes: list = field(default_factory=list)
    # Moving cylinder ("person"): waypoints (x, y) walked back and forth at `person_speed`.
    person_path: list = field(default_factory=list)
    person_speed: float = 0.8
    person_radius: float = 0.2
    person_height: float = 1.7


def room(width=10.0, depth=8.0, height=2.2, thickness=0.1):
    """Walls of a closed room centred on the origin (robot starts at the centre facing +x)."""
    w, d, h, t = width / 2, depth / 2, height / 2, thickness / 2
    grey = (0.6, 0.6, 0.65, 1.0)
    return [Box('wall_front', (w, 0, h), (t, d, h), grey), Box('wall_back', (-w, 0, h), (t, d, h), grey),
            Box('wall_left', (0, d, h), (w, t, h), grey), Box('wall_right', (0, -d, h), (w, t, h), grey)]


SCENARIOS = {
    # Static objects: the backlog's 0.35 m box ahead, a low box, a tall column, a table-height box.
    'room': Scenario('room', room() + [
        Box('box_035', (2.0, 0.0, 0.175), (0.175, 0.175, 0.175)),
        Box('low_box', (1.5, 1.5, 0.075), (0.3, 0.2, 0.075)),
        Box('column', (3.0, -1.8, 1.1), (0.15, 0.15, 1.1), (0.5, 0.5, 0.5, 1.0)),
        Box('table', (-2.5, 1.5, 0.375), (0.6, 0.4, 0.375), (0.55, 0.35, 0.2, 1.0))]),
    # A person crossing 1.5 m ahead, then walking away.
    'person': Scenario('person', room(), person_path=[(1.5, -3.0), (1.5, 3.0)]),
    # Empty room: odometry drift and free-space behaviour.
    'empty': Scenario('empty', room()),
}


def robot_xml(assets):
    path = Path(assets) / 'unitree_rl_gym/resources/robots/g1_description/g1_12dof.xml'
    if not path.is_file():
        raise FileNotFoundError(f'{path} not found; run sim/fetch_assets.sh')
    return path


def build(assets, scenario='room'):
    """Compile the world. Returns (model, scenario)."""
    scene = SCENARIOS[scenario] if isinstance(scenario, str) else scenario
    spec = mujoco.MjSpec.from_file(str(robot_xml(assets)))
    spec.option.timestep = 0.002
    pelvis = spec.body('pelvis')
    for geom in spec.geoms:
        geom.group = ROBOT_GROUP
    x, y, z, w = IMU_QUAT_XYZW
    pelvis.add_site(name='imu', pos=IMU_POS, quat=[w, x, y, z])
    pelvis.add_site(name='lidar', quat=[w, x, y, z],
                    pos=list(np.asarray(IMU_POS) + _rotate((w, x, y, z), LIDAR_IN_IMU)))
    spec.add_sensor(name='imu_acc', type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname='imu')
    spec.add_sensor(name='imu_gyro', type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname='imu')
    world = spec.worldbody
    world.add_light(pos=[0, 0, 5], dir=[0, 0, -1])
    world.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05],
                   rgba=[0.3, 0.35, 0.4, 1])
    for box in scene.boxes:
        world.add_geom(name=box.name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=list(box.pos),
                       size=list(box.size), rgba=list(box.rgba))
    if scene.person_path:
        person = world.add_body(name='person', mocap=True, pos=[*scene.person_path[0], 0])
        person.add_geom(name='person', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                        size=[scene.person_radius, scene.person_height / 2, 0],
                        pos=[0, 0, scene.person_height / 2], rgba=[0.2, 0.5, 0.9, 1],
                        contype=0, conaffinity=0)
    return spec.compile(), scene


def _rotate(quat_wxyz, vector):
    out = np.zeros(3)
    mujoco.mju_rotVecQuat(out, np.asarray(vector, float), np.asarray(quat_wxyz, float))
    return out


def person_position(scene, time):
    """Back-and-forth walk along the scenario path (x, y)."""
    points = np.asarray(scene.person_path, float)
    if len(points) < 2:
        return points[0] if len(points) else None
    segments = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = segments.sum()
    s = (time * scene.person_speed) % (2 * total)
    s = s if s <= total else 2 * total - s
    for start, length, end in zip(points[:-1], segments, points[1:]):
        if s <= length:
            return start + (end - start) * (s / length)
        s -= length
    return points[-1]
