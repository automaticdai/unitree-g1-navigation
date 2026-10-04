import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from .geometry import transform


def parameter(node, name, default):
    return node.declare_parameter(name, default).value


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def matrix_from_pose(pose):
    return transform((pose.position.x, pose.position.y, pose.position.z),
                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))


def fill_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)


def tf_message(parent, child, matrix, stamp):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id, msg.header.stamp = parent, child, stamp
    t = msg.transform.translation
    t.x, t.y, t.z = map(float, matrix[:3, 3])
    q = msg.transform.rotation
    q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(matrix[:3, :3]).as_quat())
    return msg


def xyz(msg):
    # Humble returns an iterator; newer sensor_msgs_py may return a structured array.
    points = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if isinstance(points, np.ndarray) and points.dtype.names:
        return np.column_stack([points[name].reshape(-1) for name in ('x', 'y', 'z')]).astype(float)
    return np.asarray(list(points), dtype=float).reshape(-1, 3)


def cloud(points, frame, stamp):
    return point_cloud2.create_cloud_xyz32(Header(frame_id=frame, stamp=stamp), points.tolist())


def health(node, publisher, name, ok, message, stamp=None):
    msg = DiagnosticArray()
    msg.header.stamp = stamp or node.get_clock().now().to_msg()
    msg.status = [DiagnosticStatus(name=name, level=DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR,
                                   message=message, hardware_id='g1_navigation')]
    publisher.publish(msg)


def spin(factory):
    rclpy.init()
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
