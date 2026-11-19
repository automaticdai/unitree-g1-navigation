"""Diagnostic: cluster obstacle points that are far from every true object (simulator only)."""
import sys, time
import numpy as np
import rclpy
sys.path.insert(0, '/ws/sim')
from check_teleop import Checker, GHOST_DISTANCE
rclpy.init(); n = Checker(); n.joy_enabled = False
end = time.time() + float(sys.argv[1]) if len(sys.argv) > 1 else time.time() + 40
while time.time() < end:
    n.spin(3.0)
    if n.obstacles is None or not n.shapes:
        print('waiting', flush=True); continue
    _, pts = n.obstacles
    d = n.nearest(pts)
    g = pts[d > GHOST_DISTANCE]
    person = [s for s in n.shapes.values() if s.kind == 3]
    pc = person[0].centre[:2] if person else None
    print(f'robot {np.round(n.pose[:2],2)} person {np.round(pc,2)}; obstacles {len(pts)}, ghosts {len(g)}', flush=True)
    if len(g):
        # coarse clusters on a 0.5 m grid
        keys, counts = np.unique(np.round(g[:, :2] / 0.5).astype(int), axis=0, return_counts=True)
        top = np.argsort(-counts)[:5]
        for k, c in zip(keys[top], counts[top]):
            sel = np.all(np.round(g[:, :2] / 0.5).astype(int) == k, axis=1)
            print(f'   ~({k[0]*0.5:+.1f},{k[1]*0.5:+.1f}) {c} pts, z {g[sel,2].min():.2f}-{g[sel,2].max():.2f}, '
                  f'dist to nearest object {d[d > GHOST_DISTANCE][sel].min():.2f}-{d[d > GHOST_DISTANCE][sel].max():.2f}', flush=True)
