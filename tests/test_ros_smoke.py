"""ROS-backed smoke tests; skipped by the lightweight non-ROS test environment."""
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, SetBool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from g1_navigation.command_bridge import CommandBridge
from g1_navigation.sensor_adapter import SensorAdapter
from g1_navigation.map_recorder import MapRecorder
from g1_navigation.localizer import Localizer
from g1_navigation.ros_helpers import cloud, xyz
from g1_navigation.map_io import save_bundle


@pytest.fixture(autouse=True)
def ros_context():
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_ros_cloud_and_adapter():
    rclpy.init()
    node = SensorAdapter()
    try:
        points = np.random.default_rng(2).uniform([1., -2., -1.], [3., 2., 1.], (100, 3))
        stamp = node.get_clock().now().to_msg()
        scan = cloud(points, 'body', stamp)
        np.testing.assert_allclose(xyz(scan), points, atol=1e-6)
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'camera_init', 'body'
        odom.header.stamp = stamp
        odom.pose.pose.orientation.w = 1.
        node.process(odom, scan)
        assert node.previous is not None
        # Next scan exercises velocity calculation, not just initialization.
        stamp.nanosec += 100_000_000
        if stamp.nanosec >= 1_000_000_000:
            stamp.sec += 1
            stamp.nanosec -= 1_000_000_000
        odom.pose.pose.position.x = 0.01
        node.process(odom, cloud(points, 'body', stamp))
        assert node.previous is not None
    finally:
        node.destroy_node()


def test_ros_map_save_service(tmp_path):
    directory = tmp_path / 'room'
    rclpy.init(args=['--ros-args', '-p', f'session_directory:={directory}'])
    node = MapRecorder()
    try:
        node.record(cloud(np.array([[0., 0., 0.], [1., 0., 0.5]]), 'odom', node.get_clock().now().to_msg()))
        response = node.save(Trigger.Request(), Trigger.Response())
        assert response.success, response.message
        assert (directory / 'map.pcd').exists()
    finally:
        node.destroy_node()


def test_ros_dry_bridge_never_imports_sdk_and_rejects_missing_health():
    import sys
    rclpy.init()
    node = CommandBridge()
    try:
        node.tick()
        response = node.arm(SetBool.Request(data=True), SetBool.Response())
        assert not response.success
        assert node.client is None
        assert 'unitree_sdk2py' not in sys.modules
        for name in ('perception', 'localization'):
            msg = DiagnosticArray()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.status = [DiagnosticStatus(name=name, level=DiagnosticStatus.OK)]
            node.diagnostic(name, msg)
        from std_msgs.msg import Bool
        node.operator(Bool(data=True))
        response = node.arm(SetBool.Request(data=True), SetBool.Response())
        assert response.success, response.message
        msg.header.stamp.sec -= 10
        node.diagnostic('localization', msg)
        assert not node.gate.armed
    finally:
        node.destroy_node()


def test_mapping_launch_has_no_velocity_or_navigation_nodes(tmp_path):
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch_ros.actions import Node as LaunchNode
    path = Path(__file__).resolve().parents[1] / 'src/g1_navigation/launch/bringup.launch.py'
    spec = importlib.util.spec_from_file_location('g1_bringup', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(dict(
        mode='mapping', replay='false', enable_hardware='false', start_sensors='false',
        robot_config=str(path.parent.parent / 'config/robot.yaml'),
        fastlio_config=str(path.parent.parent / 'config/fastlio.yaml'),
        session_directory=str(tmp_path / 'new_map'), map_directory='', record_bag='false'))
    actions = module.build(context)
    executables = [action.node_executable for action in actions if isinstance(action, LaunchNode)]
    assert executables == ['sensor_adapter', 'map_recorder']


def test_ros_localizer_loads_map_and_waits_for_initial_pose(tmp_path):
    points = np.random.default_rng(4).uniform([-2., -2., 0.], [2., 2., 1.8], (1000, 3))
    directory = save_bundle(tmp_path / 'room', points, {})
    rclpy.init(args=['--ros-args', '-p', f'map_directory:={directory}'])
    node = Localizer()
    try:
        node.match(cloud(points, 'odom', node.get_clock().now().to_msg()))
        assert node.correction is None
        assert node.accepted == 0
    finally:
        node.destroy_node()
