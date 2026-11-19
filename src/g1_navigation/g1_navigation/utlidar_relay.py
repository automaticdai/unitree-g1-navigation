"""Relay Unitree's onboard Mid360 topics into this stack's ROS domain, restamped.

The G1's head Mid360 is owned by Unitree's onboard software, which publishes it
on DDS domain 0 as /utlidar/cloud_livox_mid360 (PointCloud2 with per-point
`time` in ns and `ring`) and /utlidar/imu_livox_mid360 (acceleration in g).
Running livox_ros_driver2 would redirect the sensor's data stream away from
Unitree's services, so this node only listens.

- Subscribes on `source_domain_id` (default 0) through a separate rclpy context,
  so this stack's own topics (/cmd_vel, /odom, /tf ...) stay off the robot's domain.
- Restamps both streams with one ClockOffset (remote clock -> local clock).
- Converts IMU acceleration from g to m/s^2.
- Publishes /g1/lidar/points and /g1/lidar/imu for FAST-LIO.
"""
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, PointCloud2
from .ros_helpers import parameter, seconds, spin
from .timesync import ClockOffset

GRAVITY = 9.80665
CLOUD_OUT, IMU_OUT = '/g1/lidar/points', '/g1/lidar/imu'


def to_time(value):
    sec = int(value // 1)
    return Time(sec=sec, nanosec=min(int(round((value - sec) * 1e9)), 999_999_999))


class UtlidarRelay(Node):
    def __init__(self):
        super().__init__('utlidar_relay')
        self.source_domain = parameter(self, 'source_domain_id', 0)
        cloud_in = parameter(self, 'cloud_topic', '/utlidar/cloud_livox_mid360')
        imu_in = parameter(self, 'imu_topic', '/utlidar/imu_livox_mid360')
        self.acc_scale = parameter(self, 'imu_acc_scale', GRAVITY)  # g -> m/s^2
        self.clock_offset = ClockOffset(window=parameter(self, 'offset_window', 5.0),
                                        jump=parameter(self, 'offset_jump', 1.0))
        self.lock = threading.Lock()
        self.counts = {'cloud': 0, 'imu': 0}
        # Reliable outputs match both reliable (FAST-LIO) and best-effort subscribers.
        self.cloud_pub = self.create_publisher(PointCloud2, CLOUD_OUT, 5)
        self.imu_pub = self.create_publisher(Imu, IMU_OUT, 200)
        self.source_context = Context()
        rclpy.init(context=self.source_context, domain_id=self.source_domain)
        self.source = Node('utlidar_relay_source', context=self.source_context)
        # Best effort matches Unitree's reliable publishers and never back-pressures them.
        self.source.create_subscription(PointCloud2, cloud_in, self.on_cloud, qos_profile_sensor_data)
        self.source.create_subscription(Imu, imu_in, self.on_imu, QoSProfile(
            depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.source_executor = SingleThreadedExecutor(context=self.source_context)
        self.source_executor.add_node(self.source)
        self.thread = threading.Thread(target=self.source_executor.spin, daemon=True)
        self.thread.start()
        self.report = self.create_timer(5.0, self.log_rates)
        self.get_logger().info(f'relaying {cloud_in}, {imu_in} from domain {self.source_domain} '
                               f'to {CLOUD_OUT}, {IMU_OUT}')

    def restamp(self, header):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            self.clock_offset.update(seconds(header.stamp), now)
            header.stamp = to_time(self.clock_offset.local(seconds(header.stamp)))

    def on_cloud(self, msg):
        self.restamp(msg.header)
        self.cloud_pub.publish(msg)
        self.counts['cloud'] += 1

    def on_imu(self, msg):
        self.restamp(msg.header)
        a = msg.linear_acceleration
        a.x, a.y, a.z = a.x * self.acc_scale, a.y * self.acc_scale, a.z * self.acc_scale
        if not msg.header.frame_id:
            msg.header.frame_id = 'livox_imu'
        self.imu_pub.publish(msg)
        self.counts['imu'] += 1

    def log_rates(self):
        cloud, imu = self.counts['cloud'] / 5.0, self.counts['imu'] / 5.0
        self.counts = {'cloud': 0, 'imu': 0}
        offset = self.clock_offset.offset
        message = f'cloud {cloud:.1f} Hz, imu {imu:.0f} Hz, clock offset ' + (
            f'{offset:+.3f} s' if offset is not None else 'unknown')
        if cloud < 5 or imu < 100:
            self.get_logger().warning(message + ' (low or no input from the robot)')
        else:
            self.get_logger().info(message, throttle_duration_sec=30.0)

    def destroy_node(self):
        self.source_executor.shutdown()
        self.source.destroy_node()
        if self.source_context.ok():
            rclpy.shutdown(context=self.source_context)
        return super().destroy_node()


def main():
    spin(UtlidarRelay)
