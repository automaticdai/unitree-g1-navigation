"""Convert FAST-LIO IMU poses to robot poses and live obstacle observations."""
import numpy as np
from scipy.spatial.transform import Rotation
import message_filters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray
from tf2_ros import TransformBroadcaster
from .geometry import apply, transform, planar, relative_velocity
from .ros_helpers import parameter, matrix_from_pose, fill_pose, seconds, cloud, xyz, health, tf_message, spin


class SensorAdapter(Node):
    def __init__(self):
        super().__init__('sensor_adapter')
        self.calibrated = parameter(self, 'calibration_confirmed', False)
        self.mount = transform(parameter(self, 'base_to_imu_xyz', [0., 0., 1.0]),
                               parameter(self, 'base_to_imu_xyzw', [0., 0., 0., 1.]))
        self.floor_offset = parameter(self, 'initial_imu_height', 1.0)
        self.lower = np.array(parameter(self, 'self_filter_min', [-0.25, -0.35, -0.1]))
        self.upper = np.array(parameter(self, 'self_filter_max', [0.25, 0.35, 1.8]))
        self.min_z = parameter(self, 'obstacle_min_height', 0.10)
        self.max_z = parameter(self, 'obstacle_max_height', 1.9)
        self.max_range = parameter(self, 'max_range', 15.0)
        values = np.r_[self.lower, self.upper, self.floor_offset, self.min_z, self.max_z, self.max_range]
        if (not np.isfinite(values).all() or self.lower.shape != (3,) or self.upper.shape != (3,)
                or np.any(self.lower >= self.upper) or self.floor_offset <= 0
                or not 0 <= self.min_z < self.max_z or self.max_range <= 0):
            raise ValueError('Invalid floor, self-filter, obstacle height, or range configuration')
        self.previous = None
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.obstacles = self.create_publisher(PointCloud2, '/perception/obstacles', 5)
        self.scans = self.create_publisher(PointCloud2, '/mapping/points', 5)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/health/perception', 5)
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/Odometry', qos_profile=qos_profile_sensor_data)
        self.cloud_sub = message_filters.Subscriber(self, PointCloud2, '/cloud_registered_body', qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.TimeSynchronizer([self.odom_sub, self.cloud_sub], 30)
        self.sync.registerCallback(self.process)
        if not self.calibrated:
            self.get_logger().warning('Example mount geometry: perception will not authorize autonomy')

    def process(self, odom, scan):
        stamp = scan.header.stamp
        try:
            if odom.header.frame_id != 'camera_init' or odom.child_frame_id != 'body' or scan.header.frame_id != 'body':
                raise ValueError('Unexpected FAST-LIO frames; expected camera_init -> body')
            imu = matrix_from_pose(odom.pose.pose)
            imu[2, 3] += self.floor_offset
            base = imu @ np.linalg.inv(self.mount)
            flat = planar(base)
            points = xyz(scan)
            points = points[np.isfinite(points).all(axis=1)]
            if len(points) < 30:
                raise ValueError('Too few finite LiDAR returns')
            points = points[np.linalg.norm(points, axis=1) <= self.max_range]
            base_points = apply(self.mount, points)
            keep = ~np.all((base_points >= self.lower) & (base_points <= self.upper), axis=1)
            world = apply(imu, points[keep])
            if len(world) < 30:
                raise ValueError('Too few returns after body/range filtering')
            current_time = seconds(stamp)
            velocity, angular = np.zeros(3), np.zeros(3)
            valid_motion = False
            if self.previous:
                old_time, old = self.previous
                velocity, angular = relative_velocity(old, flat, current_time - old_time)
                if np.linalg.norm(velocity) > 2.0 or np.linalg.norm(angular) > 3.0:
                    raise ValueError('Odometry jump')
                valid_motion = True
            self.previous = (current_time, flat)
            self.tf.sendTransform([
                tf_message('odom', 'base_footprint', flat, stamp),
                tf_message('base_footprint', 'base_link', np.linalg.inv(flat) @ base, stamp),
                tf_message('base_link', 'mid360_imu', self.mount, stamp),
            ])
            output = Odometry()
            output.header.frame_id, output.child_frame_id, output.header.stamp = 'odom', 'base_footprint', stamp
            fill_pose(output.pose.pose, flat)
            output.twist.twist.linear.x, output.twist.twist.linear.y = map(float, velocity[:2])
            output.twist.twist.angular.z = float(angular[2])
            # Conservative nominal covariance, not a claim of calibrated uncertainty.
            for i in range(6):
                output.pose.covariance[i * 7] = 0.05
                output.twist.covariance[i * 7] = 0.1
            self.odom_pub.publish(output)
            self.scans.publish(cloud(world, 'odom', stamp))
            selected = world[(world[:, 2] >= self.min_z) & (world[:, 2] <= self.max_z)]
            self.obstacles.publish(cloud(apply(np.linalg.inv(flat), selected), 'base_footprint', stamp))
            tilt = np.linalg.norm(Rotation.from_matrix(base[:3, :3]).as_euler('xyz')[:2])
            ok = self.calibrated and valid_motion and tilt < 0.35
            health(self, self.diagnostics, 'perception', ok,
                   'fresh scan and pose' if ok else 'uncalibrated, initializing, or excessive tilt', stamp)
        except (ValueError, TypeError) as exc:
            self.previous = None
            health(self, self.diagnostics, 'perception', False, str(exc), stamp)


def main():
    spin(SensorAdapter)
