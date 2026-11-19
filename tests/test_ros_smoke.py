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


def test_ros_map_recorder_without_session_publishes_but_refuses_save():
    rclpy.init()
    node = MapRecorder()
    try:
        node.record(cloud(np.array([[0., 0., 0.], [1., 0., 0.5]]), 'odom', node.get_clock().now().to_msg()))
        node.publish_map()
        response = node.save(Trigger.Request(), Trigger.Response())
        assert not response.success
        assert 'No session_directory' in response.message
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


def _teleop_args(*extra):
    args = ['--ros-args', '-p', 'profile:=teleop']
    for item in extra:
        args += ['-p', item]
    return args


def _feed(node, name):
    msg = DiagnosticArray()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.status = [DiagnosticStatus(name=name, level=DiagnosticStatus.OK)]
    node.diagnostic(name, msg)


def test_ros_teleop_bridge_requires_operator_health_not_localization():
    rclpy.init(args=_teleop_args())
    node = CommandBridge()
    try:
        node.tick()
        assert node.cancel_clients == []
        _feed(node, 'perception')
        assert not node.arm(SetBool.Request(data=True), SetBool.Response()).success
        _feed(node, 'operator')
        response = node.arm(SetBool.Request(data=True), SetBool.Response())
        assert response.success, response.message
        assert node.arm(SetBool.Request(data=False), SetBool.Response()).success
        node.tick()
        assert not node.cancel_failed
        _feed(node, 'perception')
        _feed(node, 'operator')
        response = node.arm(SetBool.Request(data=True), SetBool.Response())
        assert response.success, response.message
    finally:
        node.destroy_node()


def test_ros_teleop_bridge_allows_bounded_reverse():
    import time
    from geometry_msgs.msg import Twist
    rclpy.init(args=_teleop_args('max_vx_reverse:=0.1'))
    node = CommandBridge()
    try:
        node.tick()
        _feed(node, 'perception')
        _feed(node, 'operator')
        assert node.arm(SetBool.Request(data=True), SetBool.Response()).success
        msg = Twist()
        msg.linear.x = -1.0
        node.command(msg)
        assert node.gate.output(time.monotonic())[0] == -0.1
    finally:
        node.destroy_node()


def test_ros_navigation_bridge_rejects_reverse_config():
    rclpy.init(args=['--ros-args', '-p', 'max_vx_reverse:=0.1'])
    with pytest.raises(ValueError):
        CommandBridge()


def _build_launch(**overrides):
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    path = Path(__file__).resolve().parents[1] / 'src/g1_navigation/launch/bringup.launch.py'
    spec = importlib.util.spec_from_file_location('g1_bringup', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    config = path.parent.parent / 'config'
    context.launch_configurations.update(dict(
        mode='mapping', replay='false', enable_hardware='false', start_sensors='false',
        robot_config=str(config / 'robot.yaml'), fastlio_config=str(config / 'fastlio.yaml'),
        teleop_config=str(config / 'teleop.yaml'),
        session_directory='', map_directory='', record_bag='false'))
    context.launch_configurations.update(overrides)
    return module.build(context)


def _executables(actions):
    from launch_ros.actions import Node as LaunchNode
    return [action.node_executable for action in actions if isinstance(action, LaunchNode)]


def test_mapping_launch_has_no_velocity_or_navigation_nodes(tmp_path):
    actions = _build_launch(mode='mapping', session_directory=str(tmp_path / 'new_map'))
    assert _executables(actions) == ['sensor_adapter', 'map_recorder']


def test_teleop_launch_graph(tmp_path):
    from launch.actions import IncludeLaunchDescription
    actions = _build_launch(mode='teleop')
    assert _executables(actions) == ['sensor_adapter', 'map_recorder', 'footprint_publisher',
                                     'collision_monitor', 'lifecycle_manager', 'gamepad_teleop',
                                     'command_bridge']
    assert not any(isinstance(action, IncludeLaunchDescription) for action in actions)
    assert _executables(_build_launch(mode='teleop', session_directory=str(tmp_path / 'new'))) == \
        _executables(actions)
    for bad in ('relative/dir', str(tmp_path)):
        with pytest.raises(ValueError):
            _build_launch(mode='teleop', session_directory=bad)


def test_teleop_hardware_replay_rejected(tmp_path):
    with pytest.raises(ValueError):
        _build_launch(mode='teleop', replay='true', enable_hardware='true')
    with pytest.raises(ValueError):
        _build_launch(mode='mapping', enable_hardware='true', session_directory=str(tmp_path / 'm'))
    _build_launch(mode='teleop', enable_hardware='true')


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


def test_ros_gamepad_teleop_wires_joy_to_command():
    import time
    from sensor_msgs.msg import Joy
    from g1_navigation.gamepad_teleop import GamepadTeleop
    rclpy.init()
    node = GamepadTeleop()
    try:
        node.tick(0.)
        assert node.last_command == (0., 0.)
        assert not node.gamepad.healthy(0.)
        msg = Joy(axes=[0.] * 8, buttons=[0] * 11)
        msg.buttons[4] = 1
        msg.axes[1] = 1.0
        node.joy(msg)
        node.tick()
        time.sleep(0.1)
        node.joy(msg)
        node.tick()
        assert node.last_command[0] > 0
    finally:
        node.destroy_node()


def test_ros_footprint_publisher_publishes_rectangle_in_base_frame():
    from g1_navigation.footprint_publisher import FootprintPublisher
    points = [0.4, 0.45, 0.4, -0.45, -0.4, -0.45, -0.4, 0.45]
    rclpy.init(args=['--ros-args', '-p', f'points:={points}'])
    node = FootprintPublisher()
    try:
        msg = node.message()
        assert msg.header.frame_id == 'base_footprint'
        assert [round(v, 6) for p in msg.polygon.points for v in (p.x, p.y)] == points
        assert node.publisher.topic_name == '/teleop/footprint'
    finally:
        node.destroy_node()


def test_ros_footprint_publisher_rejects_bad_polygon():
    rclpy.init(args=['--ros-args', '-p', 'points:=[0.4, 0.45, 0.4]'])
    from g1_navigation.footprint_publisher import FootprintPublisher
    with pytest.raises(ValueError):
        FootprintPublisher()
