"""ROS 2 front end of the G1 simulator.

Plays the part of the robot for the navigation stack:

- On `source_domain_id` (the robot's DDS domain; 0 on the real G1) it publishes
  the head Mid360 as Unitree's onboard software does:
  /utlidar/cloud_livox_mid360 and /utlidar/imu_livox_mid360 (acceleration in g),
  stamped with this host's clock plus `clock_offset` seconds (the real G1's
  clock was ~27 min off), so utlidar_relay's offset estimation is exercised.
- On the stack's own domain (ROS_DOMAIN_ID) it follows /g1/command_preview,
  the dry-run command bridge's gated output, and stops when no command has
  arrived for `command_lease` seconds (the robot-side command expiry).
  It also publishes ground truth for evaluation:
    /sim/ground_truth  nav_msgs/Odometry of base_footprint in an `odom`-aligned
                       frame (the robot's start pose; compare with /odom)
    /sim/scene         visualization_msgs/MarkerArray of the true obstacles

Runs in real time against the wall clock; it is not a /clock source.
"""
import argparse
import os
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from .engine import Simulator
from .mid360 import CLOUD_FIELDS, GRAVITY, POINT_DTYPE


def stamp(seconds):
    sec = int(seconds // 1)
    return Time(sec=sec, nanosec=min(int(round((seconds - sec) * 1e9)), 999_999_999))


def yaw_of(q):
    w, x, y, z = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def cloud_message(points, header):
    msg = PointCloud2(header=header, height=1, width=len(points), is_bigendian=False,
                      point_step=POINT_DTYPE.itemsize, is_dense=True)
    msg.fields = [PointField(name=name, offset=offset, datatype=kind, count=1)
                  for name, offset, kind in CLOUD_FIELDS]
    msg.row_step = msg.point_step * msg.width
    msg.data = points.tobytes()
    return msg


class SimNode:
    def __init__(self, args):
        self.args = args
        self.sim = Simulator(args.assets, args.scenario, seed=args.seed, noise=not args.no_noise)
        # Stack-side node (ROS_DOMAIN_ID).
        rclpy.init()
        self.node = Node('g1_sim')
        log = self.node.get_logger()
        self.lock = threading.Lock()
        self.command, self.command_time = np.zeros(3), -np.inf
        self.node.create_subscription(Twist, args.command_topic, self.on_command, 10)
        self.truth = self.node.create_publisher(Odometry, '/sim/ground_truth', 10)
        self.scene = self.node.create_publisher(MarkerArray, '/sim/scene', 1)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.spin_stack_side, daemon=True).start()
        # Robot-side node (source_domain_id), like Unitree's onboard software.
        self.robot_context = Context()
        rclpy.init(context=self.robot_context, domain_id=args.source_domain_id)
        self.robot = Node('g1_sim_robot', context=self.robot_context)
        self.cloud_pub = self.robot.create_publisher(PointCloud2, '/utlidar/cloud_livox_mid360', 5)
        self.imu_pub = self.robot.create_publisher(Imu, '/utlidar/imu_livox_mid360', 200)
        self.origin = None
        log.info(f'scenario {args.scenario}; publishing /utlidar/* on domain {args.source_domain_id} '
                 f'with clock offset {args.clock_offset:+.0f} s; following {args.command_topic} '
                 f'on domain {os.environ.get("ROS_DOMAIN_ID", "0")}')

    def spin_stack_side(self):
        try:
            self.executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass  # SIGINT/SIGTERM shut the context down; run() exits too

    def on_command(self, msg):
        with self.lock:
            self.command = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
            self.command_time = time.monotonic()

    def current_command(self):
        with self.lock:
            fresh = time.monotonic() - self.command_time <= self.args.command_lease
            return self.command.copy() if fresh else np.zeros(3)

    def footprint(self):
        """base_footprint (x, y, yaw) in the world."""
        position, quat = self.sim.base_pose()
        return position[0], position[1], yaw_of(quat)

    def to_odom(self, x, y, yaw=0.0):
        ox, oy, oyaw = self.origin
        c, s = np.cos(oyaw), np.sin(oyaw)
        dx, dy = x - ox, y - oy
        return c * dx + s * dy, -s * dx + c * dy, yaw - oyaw

    def publish_truth(self, now):
        position, quat = self.sim.base_pose()
        x, y, yaw = self.to_odom(*self.footprint())
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp(now), 'odom', 'sim_base_footprint'
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
        msg.twist.twist.linear.z = float(position[2])  # pelvis height, for diagnosing falls
        self.truth.publish(msg)

    def publish_scene(self, now):
        markers = MarkerArray()
        model, data = self.sim.model, self.sim.data
        for i in range(model.ngeom):
            body = model.geom_bodyid[i]
            if model.geom_type[i] not in (6, 5) or (body != 0 and model.body_mocapid[body] < 0):
                continue  # boxes (6) and cylinders (5) of the world and mocap bodies only
            m = Marker()
            m.header.stamp, m.header.frame_id = stamp(now), 'odom'
            m.ns, m.id, m.action = 'sim_truth', i, Marker.ADD
            m.type = Marker.CUBE if model.geom_type[i] == 6 else Marker.CYLINDER
            x, y, yaw = self.to_odom(data.geom_xpos[i][0], data.geom_xpos[i][1],
                                     np.arctan2(data.geom_xmat[i][3], data.geom_xmat[i][0]))
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(data.geom_xpos[i][2])
            m.pose.orientation.z, m.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
            size = model.geom_size[i]
            if model.geom_type[i] == 6:
                m.scale.x, m.scale.y, m.scale.z = map(float, 2 * size)
            else:
                m.scale.x = m.scale.y = float(2 * size[0])
                m.scale.z = float(2 * size[1])
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.3, 0.25
            markers.markers.append(m)
        self.scene.publish(markers)

    def run(self):
        sim, args = self.sim, self.args
        log = self.node.get_logger()
        start_wall = time.time() - sim.time
        last_scene, last_truth, behind_warned = -1.0, -1.0, 0.0
        self.origin = self.footprint()
        while rclpy.ok() and rclpy.ok(context=self.robot_context):
            out = sim.step(self.current_command())
            robot_clock = start_wall + args.clock_offset
            for t, gyro, acc in out.imu:
                msg = Imu()
                msg.header = Header(stamp=stamp(robot_clock + t), frame_id='livox_imu')
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro)
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, acc)
                msg.orientation_covariance[0] = -1.0
                self.imu_pub.publish(msg)
            for frame_start, points in out.frames:
                if frame_start is not None:
                    header = Header(stamp=stamp(robot_clock + frame_start), frame_id='livox_frame')
                    self.cloud_pub.publish(cloud_message(points, header))
            now = start_wall + sim.time
            if sim.time - last_truth >= 0.05:
                self.publish_truth(now)
                last_truth = sim.time
            if sim.time - last_scene >= (0.1 if sim.person >= 0 else 1.0):
                self.publish_scene(now)
                last_scene = sim.time
            if sim.fallen():
                log.error(f'robot fell at t={sim.time:.1f} s; resetting to the start pose')
                sim.reset()
                start_wall = time.time()
                self.origin = self.footprint()
                continue
            lag = time.time() - (start_wall + sim.time)
            if lag < 0:
                time.sleep(-lag)
            elif lag > 0.1 and time.time() - behind_warned > 5.0:
                log.warning(f'simulation {lag:.2f} s behind real time')
                behind_warned = time.time()

    def shutdown(self):
        self.executor.shutdown()
        for node, context in ((self.robot, self.robot_context), (self.node, None)):
            node.destroy_node()
            if context is None:
                rclpy.try_shutdown()
            else:
                rclpy.try_shutdown(context=context)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--assets', default=os.environ.get('G1_SIM_ASSETS', 'vendor/sim'),
                        help='directory holding unitree_rl_gym (sim/fetch_assets.sh)')
    parser.add_argument('--scenario', default='room', help='room, person or empty')
    parser.add_argument('--source-domain-id', type=int, default=0,
                        help="DDS domain for /utlidar/* (the robot's domain)")
    parser.add_argument('--clock-offset', type=float, default=1620.0,
                        help='seconds added to /utlidar stamps (robot clock skew)')
    parser.add_argument('--command-topic', default='/g1/command_preview')
    parser.add_argument('--command-lease', type=float, default=0.15,
                        help='stop if no command for this long (robot-side command expiry)')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--no-noise', action='store_true', help='disable sensor noise')
    args = parser.parse_args(argv)
    node = SimNode(args)
    try:
        node.run()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except rclpy._rclpy_pybind11.RCLError:
        if rclpy.ok() and rclpy.ok(context=node.robot_context):
            raise  # a real failure, not shutdown by a signal
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
