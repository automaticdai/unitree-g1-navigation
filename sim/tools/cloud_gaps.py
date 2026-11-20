"""Diagnostic: inter-arrival gaps and ages of a cloud topic (dropped frames show as ~0.2 s gaps)."""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
rclpy.init(); n = Node('cloud_gaps'); rows = []
def on(msg):
    now = n.get_clock().now().nanoseconds * 1e-9
    rows.append((now, now - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)))
for t in sys.argv[1].split(','):
    n.create_subscription(PointCloud2, t, on if t == sys.argv[1].split(',')[0] else (lambda m, t=t: None), qos_profile_sensor_data)
end = time.time() + float(sys.argv[2])
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.02)
r = np.array(rows); gaps = np.diff(r[:, 0])
print(f'{len(r)} msgs; age median {np.median(r[:,1]):.3f} max {r[:,1].max():.3f} (>0.3: {(r[:,1]>0.3).sum()}); '
      f'gap median {np.median(gaps):.3f} p99 {np.percentile(gaps,99):.3f} max {gaps.max():.3f} (>0.25: {(gaps>0.25).sum()})', flush=True)
