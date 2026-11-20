"""Fit the floor plane in odom from /mapping/points: a quantitative mount-calibration check.

Works on the simulator and on the real robot (stack running, robot standing on a
level floor):  python3 sim/tools/floor_fit.py /mapping/points
A correct mount gives a slope near 0 deg and z0 near 0 m; a pitch error of e deg
lifts the floor by tan(e) * range, e.g. 2.5 deg puts floor returns 3 m away over
the 0.10 m obstacle threshold.
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry

topic = sys.argv[1]
rclpy.init(); n = Node('floor_fit'); s = {}
n.create_subscription(PointCloud2, topic, lambda m: s.update(c=point_cloud2.read_points_numpy(m, field_names=('x','y','z')).reshape(-1,3)), qos_profile_sensor_data)
n.create_subscription(Odometry, '/odom', lambda m: s.update(o=m.pose.pose.position), 10)
for _ in range(5):
    t0 = time.time()
    while time.time() - t0 < 2: rclpy.spin_once(n, timeout_sec=0.05)
    c, o = s['c'], s['o']
    rel = c - [o.x, o.y, 0]
    r = np.hypot(rel[:, 0], rel[:, 1])
    floor = c[(c[:, 2] < 0.3) & (r < 4)]
    A = np.column_stack([floor[:, 0] - o.x, floor[:, 1] - o.y, np.ones(len(floor))])
    coef, *_ = np.linalg.lstsq(A, floor[:, 2], rcond=None)
    res = floor[:, 2] - A @ coef
    print(f'floor pts {len(floor)}: z0 {coef[2]:+.3f} m, slope x {np.degrees(np.arctan(coef[0])):+.2f} deg, y {np.degrees(np.arctan(coef[1])):+.2f} deg, '
          f'resid std {res.std():.3f}, >0.10 m above: {(floor[:,2] > 0.10).sum()}, r of those {np.round(np.percentile(r[(c[:,2]<0.3)&(r<4)][floor[:,2]>0.10],[5,50,95]),2) if (floor[:,2]>0.10).any() else "-"}', flush=True)
