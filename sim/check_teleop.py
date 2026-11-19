#!/usr/bin/env python3
"""Scripted checks against the simulated G1 (run by `sim/run.sh --check`).

Drives /joy like the gamepad and scores the stack against simulator ground truth
(/sim/ground_truth, /sim/scene). Both are in an `odom`-aligned frame at the robot's
start pose; /perception/obstacles is in base_footprint and is placed in that frame
with the true robot pose, so odometry drift counts against the result.

  room:   arm, drive at full stick towards the 0.35 m box 2 m ahead. The approach
          zone must stop the robot short of the box without contact; while stopped
          the box (now in the LiDAR's near-field blind zone) must stay in
          /perception/obstacles (obstacle memory); the robot must stop outside the
          freeze zone, so reversing away is allowed.
  person: stand still while a person-sized cylinder walks across 1.5 m ahead. It
          must be detected; space it left must clear within 1.5 s where a beam can
          see through, and within 2 s above the sensor where none can.
"""
import argparse
import collections
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PolygonStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import Joy, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

AXIS_LINEAR, DEADMAN_AXIS, ARM = 1, 2, 7  # config/teleop.yaml
GHOST_DISTANCE = 0.3  # m from any true object surface
LIDAR_HEIGHT = 1.30   # m, head Mid360 optical centre while walking


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Shape:
    """A /sim/scene marker: distance from points to its surface (box or vertical cylinder)."""

    def __init__(self, m):
        self.kind = m.type
        self.centre = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        self.yaw = 2 * np.arctan2(m.pose.orientation.z, m.pose.orientation.w)
        self.half = np.array([m.scale.x, m.scale.y, m.scale.z]) / 2

    def distance(self, points):
        local = points - self.centre
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        local = np.column_stack([c * local[:, 0] + s * local[:, 1], -s * local[:, 0] + c * local[:, 1], local[:, 2]])
        if self.kind == Marker.CYLINDER:
            radial = np.maximum(np.hypot(local[:, 0], local[:, 1]) - self.half[0], 0)
            vertical = np.maximum(np.abs(local[:, 2]) - self.half[2], 0)
            return np.hypot(radial, vertical)
        return np.linalg.norm(np.maximum(np.abs(local) - self.half, 0), axis=1)


class Checker(Node):
    def __init__(self):
        super().__init__('sim_check')
        self.pose = None          # (x, y, yaw) true, odom-aligned
        self.pelvis_z = None
        self.shapes = {}          # marker id -> Shape
        self.obstacles = None     # (stamp_wall, points in odom-aligned frame)
        self.obstacles_odom = None  # the same points in the stack's own odom frame
        self.odom = None          # the stack's pose estimate (x, y, yaw)
        self.scans = collections.deque(maxlen=10)  # recent /mapping/points (stack odom), ~1 s
        self.perception = False
        self.status = ''
        self.preview = 0.0
        self.front = None         # approach polygon (padded footprint) front x
        self.joy = {'lin': 0.0, 'deadman': False, 'arm': 0}
        self.joy_enabled = True
        self.joy_pub = self.create_publisher(Joy, '/joy', qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_truth, 10)
        self.create_subscription(MarkerArray, '/sim/scene', self.on_scene, 5)
        self.create_subscription(PointCloud2, '/perception/obstacles', self.on_obstacles, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/health/perception', lambda m: setattr(self, 'perception', any(
            s.name == 'perception' and s.level == DiagnosticStatus.OK for s in m.status)), 10)
        self.create_subscription(String, '/g1/autonomy_status', lambda m: setattr(self, 'status', m.data), 10)
        self.create_subscription(Twist, '/g1/command_preview', lambda m: setattr(self, 'preview', m.linear.x), 10)
        self.create_subscription(PolygonStamped, '/approach_zone', lambda m: setattr(
            self, 'front', max((p.x for p in m.polygon.points), default=None)), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', (
            m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))), 10)
        self.create_subscription(PointCloud2, '/mapping/points', lambda m: self.scans.append(
            point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)),
            qos_profile_sensor_data)
        self.create_timer(0.05, self.publish_joy)

    def on_truth(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.pelvis_z = msg.twist.twist.linear.z

    def on_scene(self, msg):
        self.shapes = {m.id: Shape(m) for m in msg.markers}

    def on_obstacles(self, msg):
        if self.pose is None:
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
        x, y, yaw = self.pose
        c, s = np.cos(yaw), np.sin(yaw)
        world = np.column_stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1], pts[:, 2]])
        self.obstacles = (time.monotonic(), world)
        if self.odom is not None:
            x, y, yaw = self.odom
            c, s = np.cos(yaw), np.sin(yaw)
            self.obstacles_odom = np.column_stack([x + c * pts[:, 0] - s * pts[:, 1],
                                                   y + s * pts[:, 0] + c * pts[:, 1], pts[:, 2]])

    def beam_nearby(self, points, tolerance_deg=0.5):
        """For points in the stack's odom frame: did a beam in the last ~1 s pass within
        tolerance of their direction from the LiDAR (the condition memory needs to clear them)?"""
        if self.odom is None or not self.scans:
            return np.zeros(len(points), bool)
        origin = np.array([self.odom[0], self.odom[1], LIDAR_HEIGHT])

        def unit(v):
            v = v - origin
            return v / np.linalg.norm(v, axis=1, keepdims=True)
        tree = cKDTree(unit(np.vstack(self.scans)))
        distance, _ = tree.query(unit(points), distance_upper_bound=np.radians(tolerance_deg))
        return np.isfinite(distance)

    def publish_joy(self):
        if not self.joy_enabled:
            return
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes, msg.buttons = [0.0] * 8, [0] * 11
        msg.axes[AXIS_LINEAR] = float(self.joy['lin'])
        msg.axes[DEADMAN_AXIS] = -1.0 if self.joy['deadman'] else 1.0
        msg.buttons[ARM] = self.joy['arm']
        self.joy_pub.publish(msg)

    # --- helpers ---
    def spin(self, duration, until=None, each=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if each:
                each()
            if until and until():
                return True
        return until is None

    def require(self, duration, until, what):
        if not self.spin(duration, until):
            raise RuntimeError(f'{what} (status: {self.status!r}, perception {self.perception})')

    def nearest(self, points):
        """Distance of each point to the nearest true object surface (inf if no scene yet)."""
        if not self.shapes or len(points) == 0:
            return np.full(len(points), np.inf)
        return np.min([shape.distance(points) for shape in self.shapes.values()], axis=0)


def wait_ready(node):
    node.require(30, lambda: node.pose is not None and node.shapes, 'no simulator ground truth (is g1_sim running?)')
    node.require(120, lambda: node.perception, 'perception never became healthy (FAST-LIO, relay, sensor_adapter)')
    node.require(10, lambda: node.front is not None, 'no approach polygon')


def arm(node):
    node.joy['deadman'] = True

    def press():
        node.joy['arm'] = int(time.monotonic() % 1.0 < 0.5)  # rising edge every second
    ok = node.spin(15, lambda: node.status.startswith('ARMED'), press)
    node.joy['arm'] = 0
    if not ok:
        raise RuntimeError(f'gamepad Start never armed the gate: {node.status!r}')


def check_room(node):
    box = min(node.shapes.values(), key=lambda s: np.linalg.norm(s.centre[:2]) if s.half[2] < 0.2 and
              abs(s.centre[1]) < 0.5 else np.inf)
    box_face = box.centre[0] - box.half[0]
    arm(node)
    node.joy['lin'] = 1.0
    track, stopped_since, last_print = [], None, 0.0
    end = time.monotonic() + 40
    # Stopped: the gate output stays ~0 while the stick is held (the policy keeps
    # stepping in place, so the true pose sways by a few cm and is not a stop signal).
    while time.monotonic() < end:
        node.spin(0.1)
        now = time.monotonic()
        x, y, _ = node.pose
        track.append((now, x, y))
        if node.pelvis_z is not None and node.pelvis_z < 0.5:
            raise RuntimeError('robot fell')
        if now - last_print >= 1.0:
            print(f'  drive: x={x:5.2f} y={y:+.2f} gate output {node.preview:.3f} m/s', flush=True)
            last_print = now
        if abs(node.preview) < 0.01 and len(track) > 30:
            stopped_since = stopped_since or now
            if now - stopped_since > 3.0:
                break
        else:
            stopped_since = None
    else:
        raise RuntimeError(f'robot never stopped while driving at full stick (x={node.pose[0]:.2f})')
    recent = [p for p in track if p[0] > track[-1][0] - 2.0]
    creep = recent[-1][1] - recent[0][1]
    x, y, _ = node.pose
    if abs(y - box.centre[1]) > box.half[1] + 0.25:
        print(f'NOTE: the walking policy drifted {y:+.2f} m sideways; the box was not dead ahead.')
    # Truth clearance from the robot's unpadded front (padded front - 0.05 m) to the box face.
    clearance = box_face - (x + node.front - 0.05)
    # Mean speed over 1 s windows (> one 0.8 s gait cycle), so step sway does not count.
    peak = max(np.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0]) for a, b in zip(track, track[10:]))
    # Memory: the box is close (inside the blind zone) yet must still be in /perception/obstacles.
    node.spin(1.0)
    stamp, points = node.obstacles
    on_box = int(np.sum(box.distance(points) < 0.1))
    results = [
        ('stopped short of the box (truth clearance from footprint front)', clearance > 0.0, f'{clearance:.2f} m'),
        ('stopped outside the padded footprint (freeze zone)', box_face - (x + node.front) > 0.0,
         f'{box_face - (x + node.front):+.2f} m'),
        ('did not stop far from it', clearance < 0.6, f'{clearance:.2f} m'),
        ('no forward creep in the last 2 s while stopped', creep < 0.05, f'{creep:+.3f} m'),
        ('box still in /perception/obstacles while in the blind zone', on_box >= 5, f'{on_box} points'),
        ('peak true speed (1 s mean) within the 0.2 m/s gamepad limit', peak < 0.23, f'{peak:.2f} m/s'),
    ]
    # Reverse away must be allowed.
    # Reverse away must be allowed: footprint_publisher's direction margin stops the robot
    # before the box reaches the freeze zone. Only a box truly inside the padded footprint
    # justifies the close-contact freeze.
    x0 = node.pose[0]
    padded_clearance = box_face - (x0 + node.front)
    node.joy['lin'] = -1.0
    node.spin(3.0)
    backed = x0 - node.pose[0]
    if backed > 0.1:
        results.append(('reverse away from the box allowed', True, f'{backed:.2f} m in 3 s'))
    else:
        frozen = padded_clearance < 0.0  # the freeze is only correct if the box is truly inside
        results.append(('reverse blocked: close-contact freeze, box inside the padded footprint'
                        if frozen else 'reverse away from the box allowed', frozen,
                        f'{backed:.2f} m in 3 s; true padded clearance {padded_clearance:+.2f} m'))
    node.joy.update(lin=0.0, deadman=False)
    node.spin(1.0)
    results.append(('deadman release: zero output, still armed',
                    node.preview == 0.0 and node.status.startswith('ARMED'), node.status))
    node.joy_enabled = False
    lost = node.spin(1.5, lambda: node.status.startswith('DISARMED'))
    results.append(('gamepad loss disarms', lost, node.status))
    return results


def check_person(node):
    """Score detection and how long points stay where the person has been.

    A remembered point's age is the time since the person's surface last came within
    GHOST_DISTANCE of it. Points the LiDAR can see through should clear within ~1 s.
    Points with no recent beam within 0.5 deg of their direction (above the upside-down
    head Mid360's upper field-of-view edge, about +4..+10 deg with the mount's roll) can
    never be seen through; obstacle_memory_above_sensor_age must expire them.
    """
    seen = samples = 0
    history = []        # (time, person x, y, radius)
    in_view_ages, edge_ages, edge_peak = [], [], 0
    end = time.monotonic() + 30
    last = None
    while time.monotonic() < end:
        node.spin(0.1)
        person = [s for s in node.shapes.values() if s.kind == Marker.CYLINDER]
        if not person:
            raise RuntimeError('person scenario has no cylinder in /sim/scene')
        now = time.monotonic()
        history.append((now, *person[0].centre[:2], person[0].half[0]))
        if node.obstacles is None or node.obstacles[0] == last:
            continue
        last, points = node.obstacles
        samples += 1
        seen += int(np.sum(person[0].distance(points) < 0.15) >= 5)
        ghosts = points[node.nearest(points) > GHOST_DISTANCE]
        if not len(ghosts):
            continue
        h = np.asarray(history)
        gap = np.hypot(ghosts[:, None, 0] - h[None, :, 1], ghosts[:, None, 1] - h[None, :, 2]) - h[None, :, 3]
        near = gap < GHOST_DISTANCE
        if not near.any():
            continue
        last_near = np.where(near.any(axis=1), h[near.shape[1] - 1 - np.argmax(near[:, ::-1], axis=1), 0], np.nan)
        ages = now - last_near
        far = node.nearest(points) > GHOST_DISTANCE
        visible = node.beam_nearby(node.obstacles_odom[far]) if node.obstacles_odom is not None and \
            len(node.obstacles_odom) == len(points) else np.zeros(len(ghosts), bool)
        valid = ~np.isnan(ages)
        in_view_ages += list(ages[valid & visible])
        edge = valid & ~visible
        edge_ages += list(ages[edge])
        edge_peak = max(edge_peak, int(edge.sum()))
    trail = float(np.percentile(in_view_ages, 95)) if in_view_ages else 0.0
    edge_max = max(edge_ages, default=0.0)
    return [
        ('person detected in most obstacle clouds', samples and seen / samples > 0.8, f'{seen}/{samples}'),
        ('vacated space clears within 1.5 s (95th pct age, points a beam can see through)', trail < 1.5,
         f'{trail:.1f} s'),
        ('ghosts no beam reaches (above the sensor) clear within 2 s', edge_max < 2.0,
         f'up to {edge_peak} points, oldest {edge_max:.1f} s' if edge_peak else 'none'),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scenario', default='room', choices=['room', 'person'])
    args = parser.parse_args()
    rclpy.init()
    node = Checker()
    try:
        wait_ready(node)
        results = check_room(node) if args.scenario == 'room' else check_person(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    width = max(len(name) for name, _, _ in results)
    for name, ok, value in results:
        label = 'WARN' if ok is None else 'PASS' if ok else 'FAIL'
        print(f'{label}  {name:<{width}}  {value}')
    failed = [name for name, ok, _ in results if ok is not None and not ok]
    print('ALL PASSED' if not failed else f'{len(failed)} FAILED')
    raise SystemExit(1 if failed else 0)


if __name__ == '__main__':
    main()
