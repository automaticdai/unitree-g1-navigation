"""One process graph per operating mode. Mapping never starts a command bridge.

Teleop runs the live-mapping pipeline with the collision monitor, gamepad and a
teleop-profile bridge, but no Nav2, map server or localizer."""
from pathlib import Path
import tempfile
import time
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from g1_navigation.configuration import TELEOP_FOOTPRINT_TOPIC, merge, padded_footprint, set_sim_time


def build(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    mode = arg('mode')
    replay, hardware = arg('replay') == 'true', arg('enable_hardware') == 'true'
    if hardware and (mode not in ('navigation', 'teleop') or replay):
        raise ValueError('Hardware commands require live navigation or teleop mode')
    share = Path(get_package_share_directory('g1_navigation'))
    robot_config, fastlio_config = arg('robot_config'), arg('fastlio_config')
    session, map_directory = arg('session_directory'), arg('map_directory')
    if mode == 'mapping' and (not Path(session).is_absolute() or Path(session).exists()):
        raise ValueError('Mapping requires a new absolute session_directory')
    if mode == 'teleop' and session and (not Path(session).is_absolute() or Path(session).exists()):
        raise ValueError('Teleop session_directory must be empty or a new absolute path')
    if mode not in ('mapping', 'teleop') and not all((Path(map_directory) / file).is_file()
                                               for file in ('map.yaml', 'points.npy', 'metadata.json')):
        raise ValueError('Provide a complete saved map_directory')
    actions = []
    common = [robot_config, {'use_sim_time': replay}]
    start_sensors = arg('start_sensors') == 'true'
    if not replay and start_sensors:
        config = Path(arg('livox_config'))
        if not config.is_file():
            raise ValueError('livox_config must be a Mid360 driver JSON with your sensor and host IPs')
        actions.append(Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
                            name='livox_lidar_publisher', output='screen', parameters=[{
                                'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
                                'publish_freq': 10.0, 'output_data_type': 0,
                                'frame_id': 'livox_frame', 'user_config_path': str(config),
                                'cmdline_input_bd_code': 'livox0000000001'}]))
    if start_sensors:
        actions.append(Node(package='fast_lio', executable='fastlio_mapping', output='screen',
             parameters=[fastlio_config, {'use_sim_time': replay}],
             # Raw frames are isolated; sensor_adapter owns the navigation TF tree.
             remappings=[('/tf', '/fastlio/tf'), ('/tf_static', '/fastlio/tf_static')]))
    actions.append(Node(package='g1_navigation', executable='sensor_adapter', output='screen', parameters=common))
    if mode in ('mapping', 'teleop'):
        actions.append(Node(package='g1_navigation', executable='map_recorder', output='screen',
                            parameters=common + [{'session_directory': session,
                                                  'robot_config': robot_config,
                                                  'fastlio_config': fastlio_config}]))
        if mode == 'teleop':
            teleop_config = arg('teleop_config')
            overrides = yaml.safe_load((share / 'config/nav2_overrides.yaml').read_text())
            actions.append(Node(package='g1_navigation', executable='footprint_publisher', output='screen',
                                parameters=[{'points': padded_footprint(overrides), 'use_sim_time': replay}]))
            footprint_topic = {'approach_zone.footprint_topic': TELEOP_FOOTPRINT_TOPIC}
            actions.extend(collision_monitor(share, replay, footprint_topic) + [
                Node(package='g1_navigation', executable='gamepad_teleop', output='screen',
                     parameters=common + [teleop_config]),
                Node(package='g1_navigation', executable='command_bridge', output='screen',
                     parameters=common + [teleop_config, {'enable_hardware': hardware}]),
            ])
        if arg('record_bag') == 'true':
            # Teleop without a session directory still records, beside a timestamped name.
            output = session + '_bag' if session else time.strftime('teleop_%Y%m%d_%H%M%S_bag')
            actions.append(ExecuteProcess(cmd=[
                'ros2', 'bag', 'record', '-o', output,
                '/livox/lidar', '/livox/imu', '/Odometry', '/cloud_registered_body',
                '/odom', '/tf', '/tf_static', '/health/perception'], output='screen'))
        return actions
    actions.append(Node(package='g1_navigation', executable='localizer', output='screen',
                        parameters=common + [{'map_directory': map_directory}]))
    actions.extend([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': str(Path(map_directory) / 'map.yaml'), 'use_sim_time': replay}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_lifecycle',
             parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': replay}]),
    ])
    if mode == 'localization':
        return actions
    nav_share = Path(get_package_share_directory('nav2_bringup'))
    defaults = yaml.safe_load((nav_share / 'params/nav2_params.yaml').read_text())
    overrides = yaml.safe_load((share / 'config/nav2_overrides.yaml').read_text())
    config = merge(defaults, overrides)
    config['bt_navigator']['ros__parameters'].update({
        'default_nav_to_pose_bt_xml': str(share / 'behavior_trees/navigate.xml'),
        'default_nav_through_poses_bt_xml': str(share / 'behavior_trees/navigate_through.xml')})
    set_sim_time(config, replay)
    with tempfile.NamedTemporaryFile(mode='w', prefix='g1-nav2-', suffix='.yaml', delete=False) as file:
        yaml.safe_dump(config, file)
        params_file = file.name
    actions.extend([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_share / 'launch/navigation_launch.py')),
                                 launch_arguments={'params_file': params_file, 'use_sim_time': str(replay).lower(),
                                                   'autostart': 'true', 'use_composition': 'False'}.items()),
    ] + collision_monitor(share, replay) + [
        Node(package='g1_navigation', executable='command_bridge', output='screen',
             parameters=common + [{'profile': 'navigation', 'enable_hardware': hardware}]),
    ])
    return actions


def collision_monitor(share, replay, overrides=None):
    """Shared avoidance (config/collision_monitor.yaml) for navigation and teleop."""
    return [
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             parameters=[str(share / 'config/collision_monitor.yaml'),
                         {'use_sim_time': replay, **(overrides or {})}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='collision_lifecycle',
             parameters=[{'autostart': True, 'node_names': ['collision_monitor'], 'use_sim_time': replay}]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('g1_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping',
                              choices=['mapping', 'localization', 'navigation', 'teleop']),
        DeclareLaunchArgument('replay', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('record_bag', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_sensors', default_value='true', choices=['true', 'false'],
                              description='Start driver/FAST-LIO; false requires externally supplied FAST-LIO topics'),
        DeclareLaunchArgument('robot_config', default_value=str(share / 'config/robot.yaml')),
        DeclareLaunchArgument('fastlio_config', default_value=str(share / 'config/fastlio.yaml')),
        DeclareLaunchArgument('teleop_config', default_value=str(share / 'config/teleop.yaml')),
        DeclareLaunchArgument('livox_config', default_value=''),
        DeclareLaunchArgument('session_directory', default_value=''),
        DeclareLaunchArgument('map_directory', default_value=''),
        OpaqueFunction(function=build),
    ])
