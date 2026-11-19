"""Teleop footprint for the collision monitor's APPROACH polygon.

Humble's (1.1.20) approach polygon takes its shape only from a PolygonStamped
footprint topic. In navigation the local costmap publishes the padded footprint;
teleop has no costmap, so this node publishes the same padded rectangle (computed
by the launch from nav2_overrides.yaml) in base_footprint.

Direction margin: in Humble the approach polygon is also the close-contact freeze
zone (any point inside zeroes every direction). With one fixed polygon the approach
stops the robot with the obstacle exactly on the freeze boundary, and a few cm of
gait sway or overshoot then freeze reversing too. So while the operator commands
forward (reverse), the front (rear) edge is pushed out by `direction_margin`: the
approach stops the robot that much earlier, and once the stick is released or
reversed the plain padded footprint leaves the obstacle `direction_margin` outside
the freeze zone. The unpadded stop zone backstop is unaffected.
"""
from geometry_msgs.msg import Point32, PolygonStamped, Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from .configuration import DIRECTION_DEADBAND, TELEOP_FOOTPRINT_TOPIC, directional_footprint
from .ros_helpers import parameter, spin


class FootprintPublisher(Node):
    def __init__(self):
        super().__init__('footprint_publisher')
        points = [float(value) for value in parameter(self, 'points', [0.0] * 8)]
        if len(points) < 6 or len(points) % 2 or not any(points):
            raise ValueError('points must be a flat [x0, y0, x1, y1, ...] polygon of at least 3 vertices')
        self.margin = float(parameter(self, 'direction_margin', 0.0))
        if not 0.0 <= self.margin <= 0.5:
            raise ValueError('direction_margin must be within [0, 0.5] m')
        self.points = points
        self.direction = 0
        self.frame_id = parameter(self, 'frame_id', 'base_footprint')
        self.publisher = self.create_publisher(PolygonStamped, TELEOP_FOOTPRINT_TOPIC, 1)
        if self.margin > 0:
            self.create_subscription(Twist, parameter(self, 'cmd_vel_topic', 'cmd_vel'), self.on_command, 10)
        # Publish on steady time so a stalled /clock cannot starve the monitor.
        self.timer = self.create_timer(0.2, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish()

    def on_command(self, msg):
        vx = msg.linear.x
        direction = 1 if vx > DIRECTION_DEADBAND else -1 if vx < -DIRECTION_DEADBAND else 0
        if direction != self.direction:
            self.direction = direction
            self.publish()  # at once, so the monitor sees the new shape with the next command

    def message(self):
        msg = PolygonStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        flat = directional_footprint(self.points, self.margin, self.direction)
        msg.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in zip(flat[0::2], flat[1::2])]
        return msg

    def publish(self):
        self.publisher.publish(self.message())


def main():
    spin(FootprintPublisher)
