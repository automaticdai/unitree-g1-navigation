"""Diagnostic: age of /perception/obstacles on arrival (now - header stamp), as the collision monitor sees it."""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init(); n = Node('cloud_age'); ages = []
def on(msg):
    ages.append(n.get_clock().now().nanoseconds * 1e-9 - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
n.create_subscription(PointCloud2, sys.argv[1] if len(sys.argv) > 1 else '/perception/obstacles', on, qos_profile_sensor_data)
end = time.time() + (float(sys.argv[2]) if len(sys.argv) > 2 else 30)
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
a = np.array(ages)
print(f'{len(a)} clouds: age median {np.median(a):.3f} s, p90 {np.percentile(a,90):.3f}, p99 {np.percentile(a,99):.3f}, max {a.max():.3f}, '
      f'> 0.3 s: {(a > 0.3).sum()}', flush=True)
