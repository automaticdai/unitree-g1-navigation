"""Teleop footprint for the collision monitor's APPROACH polygon.

Humble's (1.1.20) approach polygon takes its shape only from a PolygonStamped
footprint topic. In navigation the local costmap publishes the padded footprint;
teleop has no costmap, so this node publishes the same padded rectangle (computed
by the launch from nav2_overrides.yaml) in base_footprint at a steady rate.
"""
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from .configuration import TELEOP_FOOTPRINT_TOPIC
from .ros_helpers import parameter, spin


class FootprintPublisher(Node):
    def __init__(self):
        super().__init__('footprint_publisher')
        points = list(parameter(self, 'points', [0.0] * 8))
        if len(points) < 6 or len(points) % 2 or not any(points):
            raise ValueError('points must be a flat [x0, y0, x1, y1, ...] polygon of at least 3 vertices')
        self.points = [Point32(x=float(x), y=float(y)) for x, y in zip(points[0::2], points[1::2])]
        self.frame_id = parameter(self, 'frame_id', 'base_footprint')
        self.publisher = self.create_publisher(PolygonStamped, TELEOP_FOOTPRINT_TOPIC, 1)
        # Publish on steady time so a stalled /clock cannot starve the monitor.
        self.timer = self.create_timer(0.2, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish()

    def message(self):
        msg = PolygonStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.polygon.points = list(self.points)
        return msg

    def publish(self):
        self.publisher.publish(self.message())


def main():
    spin(FootprintPublisher)
