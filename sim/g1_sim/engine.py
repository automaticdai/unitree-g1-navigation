"""Headless G1 simulation: physics, walking policy, head Mid360 and IMU (no ROS)."""
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .mid360 import POINT_RATE, FrameBuilder, Mid360
from .policy import CONTROL_DT, DEFAULT_ANGLES, WalkingPolicy, pd_torque
from .world import IMU_HEIGHT, LEG_JOINTS, build, person_position

SENSOR_DECIMATION = 2  # physics steps per IMU sample and ray chunk: 250 Hz at dt 0.002
FRAME_PERIOD = 0.1     # 10 Hz LiDAR frames, like the Livox driver's publish_freq
TRACKING_GAIN = 6.0                   # 1/s: gain on the leaky integral of the velocity error
TRACKING_LEAK = 1.0                   # s: integral time constant (no catch-up after transients)
TRACKING_LIMIT = np.array([0.2, 0.2, 0.4])  # max correction added to the policy command
POLICY = Path(__file__).resolve().parents[1] / 'assets' / 'g1_motion_policy.npz'


@dataclass
class Outputs:
    imu: list    # [(time, gyro rad/s, acc g)]
    frames: list  # [(start time, structured points)]


class Simulator:
    def __init__(self, assets, scenario='room', seed=0, noise=True, policy=POLICY):
        self.model, self.scene = build(assets, scenario)
        self.data = mujoco.MjData(self.model)
        self.policy = WalkingPolicy(str(policy))
        self.lidar = Mid360(self.model, np.random.default_rng(seed), noise)
        self.frame = FrameBuilder()
        self.decimation = round(CONTROL_DT / self.model.opt.timestep)
        self.chunk = round(POINT_RATE * self.model.opt.timestep * SENSOR_DECIMATION)
        joints = [self.model.joint(name) for name in LEG_JOINTS]
        self.qadr = np.array([j.qposadr[0] for j in joints])
        self.vadr = np.array([j.dofadr[0] for j in joints])
        actuators = [self.model.actuator(name).id for name in LEG_JOINTS]
        self.ctrl = np.array(actuators)
        self.person = self.model.body('person').mocapid[0] if self.scene.person_path else -1
        self.calibrate_mount()
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr] = DEFAULT_ANGLES
        mujoco.mj_forward(self.model, self.data)
        self.policy.reset()
        self.target = DEFAULT_ANGLES.copy()
        self.tracking_error = np.zeros(3)
        self.steps = 0
        self.frame = FrameBuilder()
        self.next_frame = FRAME_PERIOD

    def calibrate_mount(self, settle=2.0, average=3.0):
        """Match the IMU's attitude and height relative to gravity to the real measurement.

        The real mount (robot_g1_head.yaml) was measured against gravity on the standing
        robot, so it includes the robot's posture. The policy stands with a different
        pelvis attitude; rotate/shift the IMU and LiDAR sites so that, while stepping in
        place, the IMU's up vector and height equal the configured mount.
        """
        imu, lidar = self.model.site('imu').id, self.model.site('lidar').id
        self.reset()
        steps, ups, heights = round(settle / CONTROL_DT), [], []
        for i in range(steps + round(average / CONTROL_DT)):
            self.step(np.zeros(3), sense=False)
            if i >= steps:
                ups.append(self.data.site_xmat[imu].reshape(3, 3).T @ [0, 0, 1])
                heights.append(self.data.site_xpos[imu][2])
        actual = np.mean(ups, axis=0)
        actual /= np.linalg.norm(actual)
        mount = np.zeros(9)
        mujoco.mju_quat2Mat(mount, self.model.site_quat[imu])
        wanted = mount.reshape(3, 3).T @ [0, 0, 1]  # the configured mount's up vector (pelvis level)
        # C (in the site frame) maps wanted -> actual; new site rotation R C gives up = C^T actual = wanted.
        axis = np.cross(wanted, actual)
        angle = np.arctan2(np.linalg.norm(axis), wanted @ actual)
        correction = np.array([1.0, 0, 0, 0])
        if np.linalg.norm(axis) > 1e-9:
            correction = np.concatenate([[np.cos(angle / 2)], axis / np.linalg.norm(axis) * np.sin(angle / 2)])
        quat = np.zeros(4)
        mujoco.mju_mulQuat(quat, self.model.site_quat[imu].copy(), correction)
        offset = self.model.site_pos[lidar] - self.model.site_pos[imu]
        old = np.zeros(9)
        mujoco.mju_quat2Mat(old, self.model.site_quat[imu])
        new = np.zeros(9)
        mujoco.mju_quat2Mat(new, quat)
        self.model.site_quat[imu] = self.model.site_quat[lidar] = quat
        self.model.site_pos[imu][2] += IMU_HEIGHT - np.mean(heights)
        self.model.site_pos[lidar] = self.model.site_pos[imu] + new.reshape(3, 3) @ (old.reshape(3, 3).T @ offset)
        self.mount_correction = (np.degrees(angle), IMU_HEIGHT - np.mean(heights))
        return self.mount_correction

    def measured_velocity(self):
        """(vx, vy) in the heading frame and yaw rate, from the free joint."""
        w, x, y, z = self.data.qpos[3:7]
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        c, s = np.cos(yaw), np.sin(yaw)
        vx, vy = self.data.qvel[0:2]
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, self.data.qpos[3:7])
        wz = (rotation.reshape(3, 3) @ self.data.qvel[3:6])[2]
        return np.array([c * vx + s * vy, -s * vx + c * vy, wz])

    def policy_command(self, command):
        """Velocity tracking around the RL policy (integral on the velocity error).

        The policy alone drifts ~2-3 cm/s sideways; Unitree's walking controller holds
        position on a zero command. A leaky integral (time constant TRACKING_LEAK) of
        the velocity error averages out the gait's sway and removes most steady drift,
        but forgets lag during speed changes, so the robot does not creep forward to
        "catch up" after a stop.
        """
        command = np.asarray(command, float)
        decay = 1.0 - CONTROL_DT / TRACKING_LEAK
        self.tracking_error = np.clip(self.tracking_error * decay + (command - self.measured_velocity()) * CONTROL_DT,
                                      -TRACKING_LIMIT / TRACKING_GAIN, TRACKING_LIMIT / TRACKING_GAIN)
        return command + TRACKING_GAIN * self.tracking_error

    @property
    def time(self):
        return self.data.time

    def base_pose(self):
        """Pelvis position (x, y, z) and orientation quaternion (w, x, y, z) in the world."""
        return self.data.qpos[0:3].copy(), self.data.qpos[3:7].copy()

    def fallen(self):
        return self.data.qpos[2] < 0.45

    def step(self, command, sense=True):
        """Advance one 20 ms control period with velocity command (vx, vy, wz)."""
        d = self.data
        out = Outputs([], [])
        q, dq = d.qpos[self.qadr], d.qvel[self.vadr]
        self.target = self.policy.step(d.time, d.qpos[3:7], d.qvel[3:6], q, dq, self.policy_command(command))
        for _ in range(self.decimation):
            if self.person >= 0:
                d.mocap_pos[self.person][:2] = person_position(self.scene, d.time)
            d.ctrl[self.ctrl] = pd_torque(self.target, d.qpos[self.qadr], d.qvel[self.vadr])
            mujoco.mj_step(self.model, d)
            self.steps += 1
            if sense and self.steps % SENSOR_DECIMATION == 0:
                rate = 1.0 / (self.model.opt.timestep * SENSOR_DECIMATION)
                gyro, acc = self.lidar.imu(d, rate)
                out.imu.append((d.time, gyro, acc))
                if d.time >= self.next_frame - 1e-9:
                    out.frames.append(self.frame.finish())
                    self.next_frame += FRAME_PERIOD
                points, ring = self.lidar.cast(d, self.chunk)
                self.frame.add(d.time, points, ring)
        return out
