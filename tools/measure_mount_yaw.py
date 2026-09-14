#!/usr/bin/env python3
"""Measure the lidar's mount yaw without AMCL and without the map.

Park the car squared up to a long flat wall (nose perpendicular to it), then run
this. It fits a line to the scan points hitting that wall in the raw `laser`
frame; the angle between that line and the frame's y-axis is the mount yaw.

Comparing the scan to the map cannot measure this: AMCL rotates the pose to make
the scan fit, so the error hides in the pose instead of showing up as a residual.
Nothing here touches TF.

Run it twice with the car squared up from two headings ~90 deg apart. A real
mount error reads the same both times; sloppy parking does not.
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

FOV = 40.0      # [deg] half-width of the forward window used for the fit
N_SCANS = 20


class Measure(Node):
    def __init__(self):
        super().__init__('measure_mount_yaw')
        self.scans = []
        self.create_subscription(LaserScan, 'scan', self._scan, 10)

    def _scan(self, m):
        if len(self.scans) < N_SCANS:
            self.scans.append(m)


def main():
    rclpy.init()
    n = Measure()
    for _ in range(400):
        rclpy.spin_once(n, timeout_sec=0.05)
        if len(n.scans) >= N_SCANS:
            break
    if not n.scans:
        print('no scans on /scan')
        return 1

    xs, ys = [], []
    for m in n.scans:
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = (np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
              & (np.abs(a) < np.deg2rad(FOV)))
        xs.append(r[ok] * np.cos(a[ok]))
        ys.append(r[ok] * np.sin(a[ok]))
    x, y = np.concatenate(xs), np.concatenate(ys)
    if len(x) < 50:
        print(f'only {len(x)} points within +-{FOV} deg; is the car facing a wall?')
        return 1

    # Total least squares: the wall direction is the cloud's principal axis.
    p = np.stack([x - x.mean(), y - y.mean()], axis=1)
    _, _, vt = np.linalg.svd(p, full_matrices=False)
    d = vt[0]
    # A correctly mounted lidar squared up to the wall sees it along +-y.
    theta = np.degrees(np.arctan2(d[0], abs(d[1])))

    # Perpendicular residual: a flat wall must come back flat. It will not if
    # the scan plane is tilted, and no yaw value in TF can repair that.
    resid = p @ np.array([-d[1], d[0]])
    span = float(np.ptp(p @ d))

    print(f'{len(n.scans)} scans, {len(x)} points within +-{FOV} deg, '
          f'wall span {span:.2f} m, mean range {np.hypot(x, y).mean():.2f} m')
    print(f'\n  MOUNT YAW           : {theta:+.2f} deg')
    print(f'  straightness residual: rms {resid.std() * 100:.2f} cm, '
          f'max {np.abs(resid).max() * 100:.2f} cm')
    if np.abs(resid).max() > 0.03 and span > 1.0:
        print('  -> wall does not come back straight: the scan plane is likely')
        print('     tilted (roll/pitch). A yaw value in TF will NOT fix that.')
    print(f'\nIf this repeats from a second heading, put it in bringup_launch.py:127')
    print(f'  arguments=[\'0.27\', \'0.0\', \'0.11\', \'{np.deg2rad(-theta):.4f}\', '
          f'\'0.0\', \'0.0\', \'base_link\', \'laser\']   # x y z yaw pitch roll')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
