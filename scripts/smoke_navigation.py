#!/usr/bin/env python3
"""Exercise the real Nav2 graph with synthetic FAST-LIO topics, no SDK or robot.

Run after sourcing a Humble workspace. The synthetic robot stays stationary:
this checks planning/command routing and watchdog stop, not route tracking.
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
import yaml
from ament_index_python.packages import get_package_share_directory
from g1_navigation.map_io import save_bundle
from g1_navigation.ros_helpers import cloud


def main():
    with tempfile.TemporaryDirectory(prefix='g1-smoke-') as directory:
        directory = Path(directory)
        axis = np.arange(-3., 3.01, 0.05)
        xx, yy = np.meshgrid(axis, axis)
        floor = np.column_stack((xx.ravel(), yy.ravel(), np.zeros(xx.size)))
        wall_axis, height = np.meshgrid(axis, np.arange(0.2, 1.81, 0.1))
        walls = []
        for side in (-3., 3.):
            walls.extend([
                np.column_stack((wall_axis.ravel(), np.full(wall_axis.size, side), height.ravel())),
                np.column_stack((np.full(wall_axis.size, side), wall_axis.ravel(), height.ravel()))])
        points = np.vstack([floor, *walls])
        bundle = save_bundle(directory / 'map', points, {'synthetic': True})
        share = Path(get_package_share_directory('g1_navigation'))
        config = yaml.safe_load((share / 'config/robot.yaml').read_text())
        config['/**']['ros__parameters']['calibration_confirmed'] = True
        config_path = directory / 'robot.yaml'
        config_path.write_text(yaml.safe_dump(config))
        log_path = directory / 'launch.log'
        with log_path.open('w') as log:
            launch = subprocess.Popen([
                'ros2', 'launch', 'g1_navigation', 'bringup.launch.py', 'mode:=navigation',
                'start_sensors:=false', 'enable_hardware:=false',
                f'map_directory:={bundle}', f'robot_config:={config_path}'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = Node('synthetic_navigation_probe')
            odom_pub = node.create_publisher(Odometry, '/Odometry', 10)
            scan_pub = node.create_publisher(PointCloud2, '/cloud_registered_body', 10)
            initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            operator_pub = node.create_publisher(Bool, '/g1/operator_heartbeat', 10)
            state = {'localized': False, 'perception': False, 'moving': False, 'stopped': False, 'status': ''}

            def diagnostic(msg):
                state['localized'] = any(s.name == 'localization' and s.level == DiagnosticStatus.OK for s in msg.status)

            def command(msg):
                moving = abs(msg.linear.x) + abs(msg.angular.z) > 0.001
                state['moving'] = state['moving'] or moving
                state['stopped'] = not moving

            node.create_subscription(DiagnosticArray, '/health/localization', diagnostic, 10)
            node.create_subscription(DiagnosticArray, '/health/perception',
                                     lambda msg: state.update(perception=any(
                                         s.name == 'perception' and s.level == DiagnosticStatus.OK
                                         for s in msg.status)), 10)
            node.create_subscription(Twist, '/g1/command_preview', command, 10)
            node.create_subscription(String, '/g1/autonomy_status', lambda msg: state.update(status=msg.data), 10)
            arm = node.create_client(SetBool, '/g1/arm')
            lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
            navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
            started = time.monotonic()
            last_publish, last_initial = 0., 0.
            arm_future, goal_future, loss_at = None, None, None
            lifecycle_future, active = None, False
            try:
                while time.monotonic() - started < 60.:
                    now = time.monotonic()
                    if launch.poll() is not None:
                        raise RuntimeError('Launch exited early')
                    if now - last_publish >= 0.1:
                        last_publish = now
                        operator_pub.publish(Bool(data=True))
                        if loss_at is None:
                            stamp = node.get_clock().now().to_msg()
                            odom = Odometry()
                            odom.header.frame_id, odom.child_frame_id, odom.header.stamp = 'camera_init', 'body', stamp
                            odom.pose.pose.orientation.w = 1.
                            odom_pub.publish(odom)
                            scan_pub.publish(cloud(points - [0., 0., 1.], 'body', stamp))
                    if not state['localized'] and now - last_initial > 2.:
                        last_initial = now
                        initial = PoseWithCovarianceStamped()
                        initial.header.frame_id = 'map'
                        initial.header.stamp = node.get_clock().now().to_msg()
                        initial.pose.pose.orientation.w = 1.
                        initial_pub.publish(initial)
                    if not active and lifecycle.service_is_ready():
                        if lifecycle_future is None:
                            lifecycle_future = lifecycle.call_async(GetState.Request())
                        elif lifecycle_future.done():
                            active = lifecycle_future.result().current_state.id == 3
                            lifecycle_future = None
                    if active and state['localized'] and state['perception'] and arm_future is None and arm.service_is_ready() and navigator.server_is_ready():
                        arm_future = arm.call_async(SetBool.Request(data=True))
                    if arm_future is not None and arm_future.done() and goal_future is None:
                        if not arm_future.result().success:
                            arm_future = None  # Sensor/health discovery can still be settling.
                        else:
                            goal = NavigateToPose.Goal()
                            goal.pose.header.frame_id = 'map'
                            goal.pose.header.stamp = node.get_clock().now().to_msg()
                            goal.pose.pose.position.x = 0.8
                            goal.pose.pose.orientation.w = 1.
                            goal_future = navigator.send_goal_async(goal)
                    if goal_future is not None and goal_future.done() and not goal_future.result().accepted:
                        raise RuntimeError('Navigation goal rejected')
                    if state['moving'] and loss_at is None:
                        loss_at = now
                    if loss_at is not None and now - loss_at > 1.5:
                        if not state['stopped'] or not state['status'].startswith('DISARMED'):
                            raise RuntimeError(f'Data loss failed to stop command gate: {state}')
                        print('PASS: Nav2 produced a guarded command; sensor loss latched disarmed with zero output.')
                        return
                    rclpy.spin_once(node, timeout_sec=0.01)
                raise RuntimeError(f'Graph smoke test timed out: {state}')
            except BaseException:
                print(log_path.read_text()[-18000:])
                raise
            finally:
                navigator.destroy()
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                if launch.poll() is None:
                    os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()


if __name__ == '__main__':
    main()
