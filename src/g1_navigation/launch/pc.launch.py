"""Operator PC side: gamepad (joy_node) and RViz. Needs ros-humble-joy and rviz2 on the PC;
they are deliberately not package dependencies because the robot does not run them."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    share = Path(get_package_share_directory('g1_navigation'))
    layout = LaunchConfiguration('layout').perform(context)
    device_id = int(LaunchConfiguration('joy_device_id').perform(context))
    return [
        Node(package='joy', executable='joy_node', name='joy_node', output='screen',
             parameters=[{'device_id': device_id, 'autorepeat_rate': 20.0, 'deadzone': 0.05}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', str(share / 'rviz' / f'{layout}.rviz')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='teleop', choices=['teleop', 'navigation']),
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        OpaqueFunction(function=build),
    ])
