"""Gamepad teleop: /joy -> /cmd_vel, operator health, and arm/disarm requests.

/cmd_vel is filtered downstream by the Nav2 collision monitor before
command_bridge sees it. Arm requests are sent once and never retried.
"""
from dataclasses import fields
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from .ros_helpers import health, parameter, spin
from .teleop import Gamepad, GamepadConfig


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__('gamepad_teleop')
        defaults = GamepadConfig()
        self.gamepad = Gamepad(GamepadConfig(**{
            f.name: parameter(self, f.name, getattr(defaults, f.name)) for f in fields(GamepadConfig)}))
        self.last_command = (0., 0.)
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.health_pub = self.create_publisher(DiagnosticArray, '/health/operator', 10)
        self.arm_client = self.create_client(SetBool, '/g1/arm')
        self.create_subscription(Joy, '/joy', self.joy, qos_profile_sensor_data)
        # A stalled /clock must never freeze the joystick watchdog.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.steady_clock)

    def joy(self, msg):
        for event in self.gamepad.update(msg.axes, msg.buttons, time.monotonic()):
            self.request_arm(event == 'arm')

    def request_arm(self, arm):
        if not self.arm_client.service_is_ready():
            self.get_logger().warning('/g1/arm unavailable; request dropped')
            return
        future = self.arm_client.call_async(SetBool.Request(data=arm))
        future.add_done_callback(self.arm_done)

    def arm_done(self, future):
        try:
            self.get_logger().info(f'arm response: {future.result().message}')
        except Exception as exc:
            self.get_logger().error(f'arm request failed: {exc}')

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        ok = self.gamepad.healthy(now)
        vx, wz = self.gamepad.command(now)
        self.last_command = (vx, wz)
        msg = Twist()
        msg.linear.x, msg.angular.z = float(vx), float(wz)
        self.command_pub.publish(msg)
        health(self, self.health_pub, 'operator', ok, 'gamepad ok' if ok else 'gamepad stale')


def main():
    spin(GamepadTeleop)
