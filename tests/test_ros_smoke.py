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


def _stamp(seconds):
    from builtin_interfaces.msg import Time
    return Time(sec=int(seconds), nanosec=int(round((seconds % 1) * 1e9)))


def test_ros_adapter_remembers_obstacles_until_seen_through():
    rclpy.init()
    node = SensorAdapter()  # default mount: IMU 1 m above the base, level
    published = []
    node.obstacles.publish = lambda msg: published.append(xyz(msg))
    box = np.array([[x, y, z] for x in (1.5, 1.55) for y in (-0.05, 0., 0.05) for z in (0.2, 0.3)])
    far = np.array([[4., y, z] for y in np.arange(-1., 1.01, 0.05) for z in (0.5, 1.0, 1.5)])

    def scan(t, robot_x, base_points):
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'camera_init', 'body'
        odom.header.stamp = _stamp(t)
        odom.pose.pose.position.x = robot_x
        odom.pose.pose.orientation.w = 1.
        node.process(odom, cloud(base_points - [robot_x, 0., 1.], 'body', _stamp(t)))
        return published[-1]
    try:
        scan(100.0, 0., np.vstack([box, far]))
        # Walked 0.15 m closer; this scan no longer sees the box (blind zone): still reported,
        # at its true distance from the robot.
        kept = scan(100.1, 0.15, far)
        assert np.min(np.linalg.norm(kept - [1.35, 0., 0.25], axis=1)) < 0.08
        # Beams through where the box was (ending twice as far from the sensor) clear it.
        sensor = np.array([0.15, 0., 1.])
        kept = scan(100.2, 0.15, np.vstack([far, sensor + 2. * (box - sensor)]))
        assert not np.any(np.linalg.norm(kept - [1.35, 0., 0.25], axis=1) < 0.1)
        # Pose gap: memory resets rather than trusting an old frame.
        assert np.min(np.linalg.norm(scan(100.3, 0.15, np.vstack([box, far])) - [1.35, 0., 0.25], axis=1)) < 0.08
        n = len(published)
        scan(101.5, 0.15, far)  # interval too long: rejected, nothing published
        assert len(published) == n
        kept = scan(101.6, 0.15, far)
        assert not np.any(np.linalg.norm(kept - [1.35, 0., 0.25], axis=1) < 0.1)
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
        session_directory='', map_directory='', record_bag='false',
        sensor_source='livox', source_domain_id='0', livox_config=''))
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


def test_utlidar_source_relays_instead_of_driving_the_sensor():
    actions = _build_launch(mode='teleop', start_sensors='true', sensor_source='utlidar', fastlio_config='')
    nodes = _executables(actions)
    assert nodes[:3] == ['utlidar_relay', 'fastlio_mapping', 'sensor_adapter']
    assert 'livox_ros_driver2_node' not in nodes
    from launch_ros.actions import Node as LaunchNode
    fastlio = next(a for a in actions if isinstance(a, LaunchNode) and a.node_executable == 'fastlio_mapping')
    from launch_ros.parameter_descriptions import ParameterFile
    files = [p.param_file[0].text for p in fastlio._Node__parameters if isinstance(p, ParameterFile)]
    assert len(files) == 1 and files[0].endswith('fastlio_g1.yaml'), files
    # The Livox source still requires its driver JSON.
    with pytest.raises(ValueError):
        _build_launch(mode='teleop', start_sensors='true', sensor_source='livox')


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
        msg.axes[2] = -1.0  # Left trigger fully pressed (deadman).
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


def test_ros_utlidar_relay_restamps_across_domains():
    import os
    import time
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from g1_navigation.utlidar_relay import UtlidarRelay
    own = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    source_domain = (own + 37) % 100  # stand-in for the robot's domain 0
    rclpy.init(args=['--ros-args', '-p', f'source_domain_id:={source_domain}'])
    robot_context = Context()
    rclpy.init(context=robot_context, domain_id=source_domain)
    relay = UtlidarRelay()
    robot = Node('fake_unitree', context=robot_context)
    listener = Node('relay_listener')
    received = {'imu': [], 'cloud': []}
    listener.create_subscription(Imu, '/g1/lidar/imu', received['imu'].append, qos_profile_sensor_data)
    listener.create_subscription(PointCloud2, '/g1/lidar/points', received['cloud'].append, qos_profile_sensor_data)
    imu_pub = robot.create_publisher(Imu, '/utlidar/imu_livox_mid360', 10)
    cloud_pub = robot.create_publisher(PointCloud2, '/utlidar/cloud_livox_mid360', 10)
    try:
        lag = 1617.0  # the robot's clock runs this far behind
        deadline = time.monotonic() + 10.
        while time.monotonic() < deadline and not (received['imu'] and received['cloud']):
            stamp = time.time() - lag
            msg = Imu()
            msg.header.stamp.sec, msg.header.stamp.nanosec = int(stamp), int((stamp % 1) * 1e9)
            msg.linear_acceleration.z = -1.0  # g
            imu_pub.publish(msg)
            scan = cloud(np.ones((40, 3)), 'livox_frame', msg.header.stamp)
            cloud_pub.publish(scan)
            rclpy.spin_once(listener, timeout_sec=0.05)
        assert received['imu'] and received['cloud'], 'relay did not cross domains'
        imu = received['imu'][-1]
        age = time.time() - (imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9)
        assert 0. <= age < 0.5, age
        assert imu.linear_acceleration.z == pytest.approx(-9.80665)
        assert abs(relay.clock_offset.offset - lag) < 0.5
    finally:
        relay.destroy_node()
        robot.destroy_node()
        listener.destroy_node()
        if robot_context.ok():
            rclpy.shutdown(context=robot_context)
