from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml
from g1_navigation.configuration import merge, set_sim_time

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'src/g1_navigation/config'


def test_configs_parse_and_motion_is_disabled_by_default():
    for path in CONFIG.glob('*.yaml'):
        assert isinstance(yaml.safe_load(path.read_text()), dict)
    robot = yaml.safe_load((CONFIG / 'robot.yaml').read_text())['/**']['ros__parameters']
    assert not robot['hardware_validation_complete']
    assert not robot['calibration_confirmed']
    assert robot['allowed_fsm_ids'] == [-1]
    assert 0 < robot['command_duration'] < robot['command_timeout']


def test_parameter_merge_preserves_plugin_defaults_and_sets_nested_clock():
    defaults = {'a': {'ros__parameters': {'plugin': 'x', 'speed': 1}},
                'b': {'b': {'ros__parameters': {}}}}
    result = merge(defaults, {'a': {'ros__parameters': {'speed': 0.2}}})
    set_sim_time(result, True)
    assert result['a']['ros__parameters'] == {'plugin': 'x', 'speed': 0.2, 'use_sim_time': True}
    assert result['b']['b']['ros__parameters']['use_sim_time']
    assert defaults['a']['ros__parameters']['speed'] == 1


def test_behavior_trees_have_no_autonomous_recovery_motion():
    for path in (ROOT / 'src/g1_navigation/behavior_trees').glob('*.xml'):
        tree = ET.parse(path)
        tags = {node.tag for node in tree.iter()}
        assert 'FollowPath' in tags
        assert not tags.intersection({'Spin', 'BackUp', 'DriveOnHeading'})


def test_dependencies_are_commit_pinned():
    for name in ('dependencies.repos', 'hardware.repos'):
        entries = yaml.safe_load((ROOT / name).read_text())['repositories']
        for entry in entries.values():
            assert len(entry['version']) == 40
            int(entry['version'], 16)


def _nav2_footprint(costmap='local_costmap'):
    overrides = yaml.safe_load((CONFIG / 'nav2_overrides.yaml').read_text())
    params = overrides[costmap][costmap]['ros__parameters']
    return yaml.safe_load(params['footprint']), params['footprint_padding']


def test_collision_monitor_uses_approach_without_slowdown():
    params = yaml.safe_load((CONFIG / 'collision_monitor.yaml').read_text())['collision_monitor']['ros__parameters']
    polygons = {name: params[name] for name in params['polygons']}
    approach, stop = polygons['approach_zone'], polygons['stop_zone']
    assert approach['action_type'] == 'approach'
    assert approach['time_before_collision'] == 1.2
    # Humble 1.1.20 takes the approach shape only from footprint_topic (points are ignored);
    # navigation uses the costmap's padded footprint, teleop overrides the topic in launch.
    assert approach['footprint_topic'] == '/local_costmap/published_footprint'
    assert 'points' not in approach
    # 1.1.20: approach freezes when points inside >= max_points; stop trips when > max_points.
    # Both therefore act on a single point.
    assert approach['max_points'] == 1
    assert stop['action_type'] == 'stop' and stop['max_points'] == 0
    footprint, padding = _nav2_footprint()
    assert stop['points'] == [value for point in footprint for value in point]
    assert 0 < padding <= 0.1
    assert all(polygon['action_type'] != 'slowdown' for polygon in polygons.values())
    assert params['cmd_vel_in_topic'] == 'cmd_vel' and params['cmd_vel_out_topic'] == 'cmd_vel_safe'


def test_collision_monitor_keeps_forwarding_zero_commands():
    # Humble stops republishing zeros after stop_pub_timeout; the command gate would then
    # disarm ("command stale") whenever the operator holds still.
    params = yaml.safe_load((CONFIG / 'collision_monitor.yaml').read_text())['collision_monitor']['ros__parameters']
    gate = yaml.safe_load((CONFIG / 'robot.yaml').read_text())['/**']['ros__parameters']
    assert params['stop_pub_timeout'] > 3600 * gate['command_timeout']


def test_teleop_footprint_matches_padded_nav2_footprint():
    from math import copysign
    from g1_navigation.configuration import padded_footprint
    footprint, padding = _nav2_footprint()
    assert _nav2_footprint('global_costmap') == (footprint, padding)
    overrides = yaml.safe_load((CONFIG / 'nav2_overrides.yaml').read_text())
    expected = [[x + copysign(padding, x), y + copysign(padding, y)] for x, y in footprint]
    assert padded_footprint(overrides) == pytest.approx([v for point in expected for v in point])


def test_teleop_config_limits():
    teleop = yaml.safe_load((CONFIG / 'teleop.yaml').read_text())
    robot = yaml.safe_load((CONFIG / 'robot.yaml').read_text())['/**']['ros__parameters']
    gamepad = teleop['gamepad_teleop']['ros__parameters']
    bridge = teleop['command_bridge']['ros__parameters']
    assert gamepad['max_vx_reverse'] == 0.1 and bridge['max_vx_reverse'] == 0.1
    assert gamepad['joy_timeout'] == 0.25
    assert bridge['profile'] == 'teleop'
    assert gamepad['max_vx'] == robot['max_vx'] and gamepad['max_wz'] == robot['max_wz']
    for name in ('deadzone', 'max_vx', 'max_vx_reverse', 'max_wz', 'accel_vx', 'accel_wz', 'joy_timeout'):
        assert isinstance(gamepad[name], float), name
    for name in ('axis_linear', 'axis_angular', 'deadman_axis', 'deadman_button', 'arm_button', 'disarm_button'):
        assert isinstance(gamepad[name], int), name
    assert isinstance(gamepad['deadman_threshold'], float)
    # Left trigger (Xbox via joy_node: +1 released, -1 pressed) is the deadman.
    assert gamepad['deadman_axis'] == 2 and -1. < gamepad['deadman_threshold'] < 0.
    # Left stick vertical drives, right stick horizontal turns.
    assert gamepad['axis_linear'] == 1 and gamepad['axis_angular'] == 3


def _rviz_topics(layout):
    data = yaml.safe_load((ROOT / f'src/g1_navigation/rviz/{layout}.rviz').read_text())
    manager = data['Visualization Manager']
    topics = set()

    def walk(displays):
        for display in displays:
            topic = display.get('Topic')
            if isinstance(topic, dict):
                topics.add(topic['Value'])
            walk(display.get('Displays', []))
    walk(manager['Displays'])
    return data, manager, topics


def test_rviz_layouts_reference_topics():
    text = (ROOT / 'src/g1_navigation/rviz/teleop.rviz').read_text()
    assert 'Fixed Frame: odom' in text
    _, manager, topics = _rviz_topics('teleop')
    assert manager['Global Options']['Fixed Frame'] == 'odom'
    assert {'/mapping/map_cloud', '/perception/obstacles', '/approach_zone',
            '/stop_zone', '/g1/status_marker'} <= topics
    assert '/camera/color/image_raw' in topics
    _, manager, topics = _rviz_topics('navigation')
    assert manager['Global Options']['Fixed Frame'] == 'map'
    assert {'/map', '/plan', '/local_costmap/costmap', '/approach_zone', '/stop_zone',
            '/perception/obstacles', '/g1/status_marker'} <= topics
    assert '/camera/color/image_raw' in topics
    tools = {tool['Class'] for tool in manager['Tools']}
    assert {'rviz_default_plugins/SetInitialPose', 'rviz_default_plugins/SetGoal'} <= tools


def test_g1_head_mount_matches_measured_up_vector():
    import numpy as np
    from g1_navigation.geometry import transform
    head = yaml.safe_load((CONFIG / 'robot_g1_head.yaml').read_text())['/**']['ros__parameters']
    base = yaml.safe_load((CONFIG / 'robot.yaml').read_text())['/**']['ros__parameters']
    assert set(head) == set(base) and not head['calibration_confirmed']
    for key in set(base) - {'base_to_imu_xyz', 'base_to_imu_xyzw', 'initial_imu_height'}:
        assert head[key] == base[key], key
    mount = transform(head['base_to_imu_xyz'], head['base_to_imu_xyzw'])
    up = np.array([-0.0039, 0.0565, -0.9984])
    assert mount[:3, :3] @ (up / np.linalg.norm(up)) == pytest.approx([0, 0, 1], abs=2e-3)
    forward = mount[:3, :3] @ [1., 0., 0.]
    assert np.degrees(np.arctan2(forward[1], forward[0])) == pytest.approx(0., abs=1.)
    assert head['base_to_imu_xyz'][2] == head['initial_imu_height'] == 1.259
