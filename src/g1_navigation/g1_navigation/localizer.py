import time
from pathlib import Path
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from .geometry import planar
from .registration import map_tree, register
from .ros_helpers import parameter, matrix_from_pose, health, seconds, xyz, tf_message, spin


class Localizer(Node):
    def __init__(self):
        super().__init__('localizer')
        bundle = Path(parameter(self, 'map_directory', ''))
        self.tree = map_tree(np.load(bundle / 'points.npy', allow_pickle=False))
        self.min_fitness = parameter(self, 'min_fitness', 0.65)
        self.max_rmse = parameter(self, 'max_rmse', 0.15)
        if not 0 < self.min_fitness <= 1 or not np.isfinite(self.max_rmse) or self.max_rmse <= 0:
            raise ValueError('Invalid localization acceptance thresholds')
        self.correction = None
        self.latest_odom = None
        self.accepted = 0
        self.last_run = -float('inf')
        self.tf = TransformBroadcaster(self)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/health/localization', 5)
        self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialize, 1)
        self.create_subscription(PointCloud2, '/mapping/points', self.match, qos_profile_sensor_data)

    def odometry(self, msg):
        self.latest_odom = msg

    def initialize(self, msg):
        self.correction, self.accepted = None, 0
        try:
            if msg.header.frame_id != 'map' or self.latest_odom is None:
                raise ValueError('Initial pose needs map frame and live odometry')
            age = self.get_clock().now().nanoseconds * 1e-9 - seconds(self.latest_odom.header.stamp)
            if not 0 <= age <= 0.5:
                raise ValueError('Odometry is stale')
            self.correction = planar(matrix_from_pose(msg.pose.pose)) @ np.linalg.inv(
                planar(matrix_from_pose(self.latest_odom.pose.pose)))
            health(self, self.diagnostics, 'localization', False, 'initializing; hold robot stationary')
        except ValueError as exc:
            health(self, self.diagnostics, 'localization', False, str(exc))

    def match(self, msg):
        now = time.monotonic()
        if now - self.last_run < 0.2:
            return
        self.last_run = now
        try:
            if self.correction is None:
                raise ValueError('Set a nearby initial pose in RViz; global relocalization is not implemented')
            if msg.header.frame_id != 'odom':
                raise ValueError('Expected scan in odom')
            points = xyz(msg)
            # Remove floor; structural features carry horizontal localization information.
            points = points[(points[:, 2] > 0.15) & (points[:, 2] < 2.5)]
            points = points[::max(1, len(points) // 4000)]
            candidate, fitness, rmse = register(points, self.tree, self.correction)
            delta = np.linalg.inv(self.correction) @ candidate
            shift = np.linalg.norm(delta[:2, 3])
            angle = abs(np.arctan2(delta[1, 0], delta[0, 0]))
            if fitness < self.min_fitness or rmse > self.max_rmse:
                raise ValueError(f'Map match rejected: overlap={fitness:.2f}, rmse={rmse:.3f}')
            # First alignment may move up to the local ICP capture range.
            if self.accepted and (shift > 0.25 or angle > 0.15):
                raise ValueError('Map correction jumped; reinitialize localization')
            self.correction = candidate
            self.accepted += 1
            self.tf.sendTransform(tf_message('map', 'odom', candidate, msg.header.stamp))
            health(self, self.diagnostics, 'localization', self.accepted >= 3,
                   f'overlap={fitness:.2f}, rmse={rmse:.3f}', msg.header.stamp)
        except ValueError as exc:
            self.accepted = 0
            health(self, self.diagnostics, 'localization', False, str(exc), msg.header.stamp)


def main():
    spin(Localizer)
