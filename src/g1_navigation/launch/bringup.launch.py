"""One process graph per operating mode. Mapping never starts a command bridge."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from g1_navigation.configuration import merge, set_sim_time


def build(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    mode = arg('mode')
    replay, hardware = arg('replay') == 'true', arg('enable_hardware') == 'true'
    if hardware and (mode != 'navigation' or replay):
        raise ValueError('Hardware commands require live navigation mode')
    share = Path(get_package_share_directory('g1_navigation'))
    robot_config, fastlio_config = arg('robot_config'), arg('fastlio_config')
    session, map_directory = arg('session_directory'), arg('map_directory')
    if mode == 'mapping' and (not Path(session).is_absolute() or Path(session).exists()):
        raise ValueError('Mapping requires a new absolute session_directory')
    if mode != 'mapping' and not all((Path(map_directory) / file).is_file()
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
    if mode == 'mapping':
        actions.append(Node(package='g1_navigation', executable='map_recorder', output='screen',
                            parameters=common + [{'session_directory': session,
                                                  'robot_config': robot_config,
                                                  'fastlio_config': fastlio_config}]))
        if arg('record_bag') == 'true':
            actions.append(ExecuteProcess(cmd=[
                'ros2', 'bag', 'record', '-o', session + '_bag',
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
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             parameters=[str(share / 'config/collision_monitor.yaml'), {'use_sim_time': replay}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='collision_lifecycle',
             parameters=[{'autostart': True, 'node_names': ['collision_monitor'], 'use_sim_time': replay}]),
        Node(package='g1_navigation', executable='command_bridge', output='screen',
             parameters=common + [{'enable_hardware': hardware}]),
    ])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('g1_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', choices=['mapping', 'localization', 'navigation']),
        DeclareLaunchArgument('replay', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('record_bag', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_sensors', default_value='true', choices=['true', 'false'],
                              description='Start driver/FAST-LIO; false requires externally supplied FAST-LIO topics'),
        DeclareLaunchArgument('robot_config', default_value=str(share / 'config/robot.yaml')),
        DeclareLaunchArgument('fastlio_config', default_value=str(share / 'config/fastlio.yaml')),
        DeclareLaunchArgument('livox_config', default_value=''),
        DeclareLaunchArgument('session_directory', default_value=''),
        DeclareLaunchArgument('map_directory', default_value=''),
        OpaqueFunction(function=build),
    ])
