#!/usr/bin/env python3
"""Exercise the teleop graph end to end with synthetic FAST-LIO topics and /joy.

Run after sourcing a Humble workspace. No SDK or robot: the synthetic robot is
stationary at the odom origin and a wall is placed in front of it. Checks the
approach slowdown, unreduced reverse away from the wall, deadman release, the
close-contact freeze, and joystick loss disarming the gate.
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, PointCloud2
from std_msgs.msg import String
import yaml
from ament_index_python.packages import get_package_share_directory
from g1_navigation.configuration import padded_footprint
from g1_navigation.ros_helpers import cloud

# Must match config/teleop.yaml.
AXIS_LINEAR, DEADMAN_AXIS, ARM, MAX_VX, MAX_VX_REVERSE = 1, 2, 7, 0.2, 0.1


def scene(wall_x):
    """Floor plus a wall across the robot's path at x = wall_x (base/odom frame)."""
    axis = np.arange(-2., 2.01, 0.05)
    xx, yy = np.meshgrid(axis, axis)
    floor = np.column_stack((xx.ravel(), yy.ravel(), np.zeros(xx.size)))
    wy, wz = np.meshgrid(np.arange(-0.6, 0.601, 0.05), np.arange(0.2, 1.51, 0.1))
    wall = np.column_stack((np.full(wy.size, wall_x), wy.ravel(), wz.ravel()))
    return np.vstack([floor, wall])


class Probe(Node):
    def __init__(self):
        super().__init__('synthetic_teleop_probe')
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 10)
        self.scan_pub = self.create_publisher(PointCloud2, '/cloud_registered_body', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy', qos_profile_sensor_data)
        self.state = {'perception': False, 'status': '', 'cmd_vel': None, 'safe': None, 'preview': None,
                      'approach': (0., 0.)}
        self.create_subscription(DiagnosticArray, '/health/perception', lambda msg: self.state.update(
            perception=any(s.name == 'perception' and s.level == DiagnosticStatus.OK for s in msg.status)), 10)
        self.create_subscription(String, '/g1/autonomy_status', lambda msg: self.state.update(status=msg.data), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda msg: self.state.update(cmd_vel=msg.linear.x), 10)
        self.create_subscription(Twist, '/cmd_vel_safe', lambda msg: self.state.update(safe=msg.linear.x), 10)
        self.create_subscription(Twist, '/g1/command_preview',
                                 lambda msg: self.state.update(preview=msg.linear.x), 10)
        self.create_subscription(PolygonStamped, '/approach_zone', lambda msg: self.state.update(approach=(
            max((abs(p.x) for p in msg.polygon.points), default=0.),
            max((abs(p.y) for p in msg.polygon.points), default=0.))), QoSProfile(depth=10))
        self.points = scene(0.55)
        self.joy = {'lin': 0., 'deadman': 0, 'arm': 0}
        self.joy_enabled = True
        self.last_scan = self.last_joy = 0.

    def publish(self, now):
        if now - self.last_scan >= 0.1:
            self.last_scan = now
            stamp = self.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id, odom.header.stamp = 'camera_init', 'body', stamp
            odom.pose.pose.orientation.w = 1.
            self.odom_pub.publish(odom)
            # Body (Mid360 IMU) frame sits 1 m above the base in robot.yaml.
            self.scan_pub.publish(cloud(self.points - [0., 0., 1.], 'body', stamp))
        if self.joy_enabled and now - self.last_joy >= 0.05:
            self.last_joy = now
            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [0.] * 8
            msg.buttons = [0] * 11
            msg.axes[AXIS_LINEAR] = float(self.joy['lin'])
            # Left trigger: +1 released, -1 fully pressed.
            msg.axes[DEADMAN_AXIS] = -1. if self.joy['deadman'] else 1.
            msg.buttons[ARM] = self.joy['arm']
            self.joy_pub.publish(msg)


class Smoke:
    def __init__(self, launch, probe):
        self.launch, self.probe = launch, probe

    def spin(self, duration, until=None, on_tick=None):
        """Spin and publish for up to duration; return True as soon as until() holds."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if self.launch.poll() is not None:
                raise RuntimeError('Launch exited early')
            now = time.monotonic()
            if on_tick:
                on_tick(now)
            self.probe.publish(now)
            rclpy.spin_once(self.probe, timeout_sec=0.01)
            if until is not None and until():
                return True
        return False

    def hold(self, duration, condition, message):
        """condition() must stay true for the whole duration."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.spin(0.02)
            if not condition():
                raise RuntimeError(f'{message}: {self.probe.state}')

    def require(self, duration, until, message):
        if not self.spin(duration, until):
            raise RuntimeError(f'{message}: {self.probe.state}')


def near(value, target, tolerance):
    return value is not None and abs(value - target) <= tolerance


def run(smoke, approach_extent):
    probe, s = smoke.probe, smoke.probe.state
    joy = probe.joy

    # 1. Perception healthy and the approach polygon fed by footprint_publisher; then arm with the gamepad.
    smoke.require(30., lambda: s['perception'] and np.allclose(s['approach'], approach_extent, atol=1e-3),
                  'Perception or approach polygon never became ready')
    joy.update(deadman=1)

    def toggle_start(now):
        # Gate health can still be settling: press Start again (rising edge) every second.
        joy['arm'] = int((now % 1.0) < 0.5)
    armed = smoke.spin(15., lambda: s['status'].startswith('ARMED'), toggle_start)
    joy['arm'] = 0
    if not armed:
        raise RuntimeError(f'Gamepad Start never armed the gate: {s}')
    smoke.hold(1., lambda: s['status'].startswith('ARMED'), 'Gate disarmed after arming')

    # 2. Full forward towards the wall at 0.55 m: approach slows the command but does not stop it.
    joy['lin'] = 1.
    smoke.require(5., lambda: near(s['cmd_vel'], MAX_VX, 1e-3), 'Gamepad never ramped to max_vx')
    smoke.spin(0.5)
    forward_safe = s['safe']
    if forward_safe is None or not 0. < forward_safe < MAX_VX - 0.02:
        raise RuntimeError(f'Approach did not slow forward motion towards the wall: {s}')
    if not near(s['preview'], forward_safe, 0.02):
        raise RuntimeError(f'Gate output does not follow the collision monitor: {s}')

    # 3. Full reverse, away from the wall: must not be reduced.
    joy['lin'] = -1.
    if not smoke.spin(3., lambda: near(s['preview'], -MAX_VX_REVERSE, 0.01)):
        if near(s['cmd_vel'], -MAX_VX_REVERSE, 1e-3):
            raise RuntimeError(f'STOP AND REPORT: Humble APPROACH reduces reverse motion away from obstacle: {s}')
        raise RuntimeError(f'Reverse command never reached the gate: {s}')
    reverse_preview = s['preview']

    # 4. Deadman released: zero output, still armed.
    joy.update(lin=0., deadman=0)
    smoke.require(1., lambda: near(s['preview'], 0., 1e-6), 'Deadman release did not zero the output')
    smoke.hold(1., lambda: s['status'].startswith('ARMED') and near(s['preview'], 0., 1e-6),
               'Deadman release disarmed or moved the robot')

    # 4b. Wall within the direction margin (outside the padded footprint, inside the forward-
    # extended one): forward is held at zero, but reversing away is not frozen.
    probe.points = scene(0.45)
    smoke.spin(1.)
    joy.update(lin=1., deadman=1)
    smoke.require(3., lambda: near(s['cmd_vel'], MAX_VX, 1e-3), 'Gamepad never ramped to max_vx (margin)')
    smoke.hold(1., lambda: near(s['preview'], 0., 1e-6), 'Forward motion allowed inside the direction margin')
    joy['lin'] = -1.
    if not smoke.spin(3., lambda: near(s['preview'], -MAX_VX_REVERSE, 0.01)):
        raise RuntimeError(f'Reverse frozen with the wall inside only the direction margin: {s}')
    joy.update(lin=0., deadman=0)
    smoke.spin(0.5)

    # 5. Wall inside the approach margin (outside stop zone and self-filter): close-contact freeze.
    probe.points = scene(0.38)
    smoke.spin(1.)  # Let the new cloud reach the collision monitor.
    joy.update(lin=1., deadman=1)
    smoke.require(3., lambda: near(s['cmd_vel'], MAX_VX, 1e-3), 'Gamepad never ramped to max_vx near the wall')
    smoke.hold(1., lambda: near(s['preview'], 0., 1e-6), 'Forward motion allowed inside the approach margin')
    joy['lin'] = -1.
    smoke.require(3., lambda: near(s['cmd_vel'], -MAX_VX_REVERSE, 1e-3), 'Gamepad never ramped to reverse')
    smoke.spin(1.)
    frozen_reverse = s['preview']
    joy.update(lin=0.)

    # 6. Joystick lost: gamepad health goes stale and the gate disarms with zero output.
    if not s['status'].startswith('ARMED'):
        raise RuntimeError(f'Gate disarmed before the joystick-loss check: {s}')
    probe.joy_enabled = False
    smoke.require(1., lambda: s['status'].startswith('DISARMED') and near(s['preview'], 0., 1e-6),
                  'Joystick loss did not disarm the gate')

    print(f'PASS: approach slowed forward to {forward_safe:.3f} m/s (wall 0.15 m ahead); '
          f'wall in the direction margin held forward at zero and allowed reverse; '
          f'reverse away passed at {reverse_preview:.3f} m/s; deadman release held zero while armed; '
          f'close contact (wall 0.38 m) froze forward, reverse gave {frozen_reverse:.3f} m/s; '
          f'joystick loss disarmed ({s["status"]}).')


def main():
    with tempfile.TemporaryDirectory(prefix='g1-teleop-smoke-') as directory:
        directory = Path(directory)
        share = Path(get_package_share_directory('g1_navigation'))
        config = yaml.safe_load((share / 'config/robot.yaml').read_text())
        config['/**']['ros__parameters']['calibration_confirmed'] = True
        config_path = directory / 'robot.yaml'
        config_path.write_text(yaml.safe_dump(config))
        overrides = yaml.safe_load((share / 'config/nav2_overrides.yaml').read_text())
        footprint = padded_footprint(overrides)
        approach_extent = (max(abs(x) for x in footprint[0::2]), max(abs(y) for y in footprint[1::2]))
        log_path = directory / 'launch.log'
        with log_path.open('w') as log:
            launch = subprocess.Popen([
                'ros2', 'launch', 'g1_navigation', 'bringup.launch.py', 'mode:=teleop',
                'start_sensors:=false', 'enable_hardware:=false', 'record_bag:=false',
                f'robot_config:={config_path}'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            probe = Probe()
            try:
                run(Smoke(launch, probe), approach_extent)
            except BaseException:
                print(log_path.read_text()[-18000:])
                raise
            finally:
                probe.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                if launch.poll() is None:
                    os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()


if __name__ == '__main__':
    main()
