"""Only node allowed to send autonomous G1 velocity RPCs.

Hardware commands are bounded leases. No stand-up, mode switch, damping, or
torque-off is issued here. SDK transport is never imported in dry-run mode.
"""
import math
import threading
import time

from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from .ros_helpers import parameter, seconds, spin
from .safety import CommandGate, Limits


class CommandBridge(Node):
    def __init__(self):
        super().__init__('command_bridge')
        self.hardware = parameter(self, 'enable_hardware', False)
        self.lease = parameter(self, 'command_duration', 0.15)
        self.lock = threading.RLock()
        self.gate = CommandGate(Limits(
            max_vx=parameter(self, 'max_vx', 0.2),
            max_wz=parameter(self, 'max_wz', 0.3),
            command_timeout=parameter(self, 'command_timeout', 0.25),
            health_timeout=parameter(self, 'health_timeout', 0.75)))
        self.client = None
        self.worker_stop = threading.Event()
        self.was_armed = False
        self.cancel_futures = []
        self.cancel_failed = False
        self.preview = self.create_publisher(Twist, '/g1/command_preview', 10)
        self.status = self.create_publisher(String, '/g1/autonomy_status', 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.command, 1)
        for name in ('perception', 'localization'):
            self.create_subscription(DiagnosticArray, f'/health/{name}',
                                     lambda msg, n=name: self.diagnostic(n, msg), 1)
        self.create_service(SetBool, '/g1/arm', self.arm)
        self.create_service(Trigger, '/g1/stop', self.stop)
        self.cancel_clients = [self.create_client(CancelGoal, f'/{action}/_action/cancel_goal')
                               for action in ('navigate_to_pose', 'navigate_through_poses')]
        if self.hardware:
            self.init_hardware()
        else:
            self.create_subscription(Bool, '/g1/operator_heartbeat', self.operator, 1)
            self.get_logger().warning('DRY RUN: no Unitree SDK connection or robot commands')
        # A stalled /clock must never freeze the command watchdog.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.steady_clock)

    def init_hardware(self):
        validated = parameter(self, 'hardware_validation_complete', False)
        interface = parameter(self, 'network_interface', '')
        allowed = parameter(self, 'allowed_fsm_ids', [-1])
        if self.get_parameter('use_sim_time').value or not validated or not interface or allowed == [-1]:
            raise ValueError('Hardware requires real time, network_interface, verified allowed_fsm_ids, '
                             'and hardware_validation_complete=true')
        if not math.isfinite(self.lease) or not 0.05 <= self.lease <= 0.25:
            raise ValueError('command_duration must be between 0.05 and 0.25 seconds')
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
        ChannelFactoryInitialize(0, interface)
        self.client = LocoClient()
        self.client.SetTimeout(0.1)
        self.client.Init()
        self.remote = ChannelSubscriber('rt/wirelesscontroller', WirelessController_)
        self.remote.Init(self.remote_input, 1)
        # Separate client/thread: status RPC cannot hold up the velocity loop.
        status_client = LocoClient()
        status_client.SetTimeout(0.1)
        status_client.Init()

        def poll():
            while not self.worker_stop.is_set():
                try:
                    code, fsm = status_client.GetFsmId()
                    valid = code == 0 and fsm in allowed
                except Exception:
                    valid = False
                with self.lock:
                    self.gate.update_health('robot', valid, time.monotonic())
                self.worker_stop.wait(0.2)
        self.worker = threading.Thread(target=poll, daemon=True)
        self.worker.start()

    def remote_input(self, msg):
        axes = (msg.lx, msg.ly, msg.rx, msg.ry)
        neutral = msg.keys == 0 and all(math.isfinite(v) and abs(v) < 0.1 for v in axes)
        with self.lock:
            self.gate.update_health('operator', neutral, time.monotonic())
            if not neutral:
                self.gate.stop('handheld controller takeover')

    def operator(self, msg):
        with self.lock:
            self.gate.update_health('operator', msg.data, time.monotonic())

    def diagnostic(self, name, msg):
        age = self.get_clock().now().nanoseconds * 1e-9 - seconds(msg.header.stamp)
        matching = [s for s in msg.status if s.name == name]
        valid = 0 <= age <= self.gate.limits.health_timeout and len(matching) == 1 and matching[0].level == DiagnosticStatus.OK
        with self.lock:
            self.gate.update_health(name, valid, time.monotonic())

    def command(self, msg):
        with self.lock:
            if any(v != 0.0 for v in (msg.linear.z, msg.angular.x, msg.angular.y)):
                self.gate.stop('unsupported velocity components')
            else:
                self.gate.receive((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

    def arm(self, request, response):
        with self.lock:
            if request.data and (self.cancel_futures or self.cancel_failed):
                response.success, response.message = False, 'Nav2 goal cancellation pending/failed; restart navigation after resolving it'
                return response
            if request.data:
                response.success = self.gate.arm(time.monotonic())
                # Remember arming even if a health callback faults before the next timer.
                self.was_armed = self.was_armed or response.success
            else:
                self.gate.stop('operator disarm')
                response.success = True
            response.message = self.gate.reason
        return response

    def stop(self, request, response):
        with self.lock:
            self.gate.stop()
        response.success, response.message = True, 'autonomy disarmed'
        return response

    def tick(self):
        with self.lock:
            if not self.hardware:
                self.gate.update_health('robot', True, time.monotonic())
            output = self.gate.output(time.monotonic())
            armed = self.gate.armed
            transition = self.was_armed and not armed
            self.was_armed = armed
            reason = self.gate.reason
        # Send one bounded zero command on disarm; do not keep fighting manual control.
        if self.client is not None and (armed or transition):
            try:
                code = self.client.SetVelocity(*output, duration=self.lease)
                if code != 0:
                    raise RuntimeError(f'velocity RPC returned {code}')
            except Exception as exc:
                with self.lock:
                    self.gate.stop(f'SDK failure: {exc}')
                self.get_logger().error(str(exc))
        if transition:
            for client in self.cancel_clients:
                if client.service_is_ready():
                    self.cancel_futures.append((time.monotonic(), client.call_async(CancelGoal.Request())))
                else:
                    self.cancel_failed = True
        remaining = []
        for started, future in self.cancel_futures:
            if future.done():
                try:
                    if future.result().return_code != 0:
                        self.cancel_failed = True
                except Exception:
                    self.cancel_failed = True
            elif time.monotonic() - started > 2.0:
                self.cancel_failed = True
            else:
                remaining.append((started, future))
        self.cancel_futures = remaining
        preview = Twist()
        preview.linear.x, preview.linear.y, preview.angular.z = output
        self.preview.publish(preview)
        self.status.publish(String(data=f'{"ARMED" if armed else "DISARMED"}: {reason}'))

    def destroy_node(self):
        self.worker_stop.set()
        if self.client is not None and (self.was_armed or self.gate.armed):
            try:
                self.client.SetVelocity(0., 0., 0., duration=self.lease)
            except Exception:
                pass  # The robot-side command lease is the process/network-failure fallback.
        return super().destroy_node()


def main():
    spin(CommandBridge)
