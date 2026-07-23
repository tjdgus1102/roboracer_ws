#!/usr/bin/env python3
"""MPPI 자율주행을 시작해도 되는 상태인지 확인한다.

현재 차량 위치(map->base_link tf), 경로상 최근접점, 진행 방향 차이,
AMCL 위치 불확실성(1시그마)을 출력하고 출발 가능 여부를 판정한다.

    python3 check_pose.py [경로CSV]

기본 경로는 paths/track_real_lap3.csv.
브링업과 localization_launch.py 가 떠 있는 상태에서 실행할 것.
"""
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # noqa: F401  (rclpy 메시지 타입 로딩용)
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

BASE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH = os.path.join(BASE, 'paths', 'track_real_lap3.csv')

# 출발 판정 기준
LIM_DIST = 0.30      # 경로까지 거리 [m]
LIM_HEADING = 20.0   # 경로 방향과의 차이 [deg]
LIM_SIGMA_XY = 0.10  # AMCL 1시그마 x, y [m]
LIM_SIGMA_YAW = 6.0  # AMCL 1시그마 yaw [deg]


class PoseCheck(Node):
    def __init__(self):
        super().__init__('check_pose')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.cov = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

    def _amcl_cb(self, msg):
        self.cov = msg.pose.covariance

    def lookup(self):
        t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        return x, y, yaw


def main():
    path_file = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    path = np.loadtxt(path_file, delimiter=',', comments='#')

    rclpy.init()
    node = PoseCheck()

    pose = None
    last_error = None
    for _ in range(60):
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            pose = node.lookup()
        except Exception as exc:
            last_error = exc
            continue
        if node.cov is not None:
            break

    if pose is None:
        print('map -> base_link tf 를 받지 못했습니다.  (%s)' % last_error)
        print('  - 브링업이 떠 있는지')
        print('  - 조이패드로 조향을 한 번 움직였는지 (/odom 트리거)')
        print('  - localization_launch.py 가 떠 있고 초기 위치를 찍었는지')
        rclpy.shutdown()
        return 1

    x, y, yaw = pose
    idx = int(np.argmin(np.linalg.norm(path[:, :2] - np.array([x, y]), axis=1)))
    dist = float(np.linalg.norm(path[idx, :2] - np.array([x, y])))
    hdiff = math.degrees((path[idx, 2] - yaw + math.pi) % (2 * math.pi) - math.pi)

    print('현재 차량   x=%.3f  y=%.3f  yaw=%.1f도' % (x, y, math.degrees(yaw)))
    print('최근접 경로점  idx %d / %d   (%.3f, %.3f)   경로방향 %.1f도'
          % (idx, len(path), path[idx, 0], path[idx, 1], math.degrees(path[idx, 2])))
    print()

    ok = True

    def check(name, value, limit, unit):
        nonlocal ok
        good = abs(value) <= limit
        ok = ok and good
        print('  %-22s %7.2f %-2s   %s'
              % (name, value, unit, 'OK' if good else 'X   (기준 %.2f)' % limit))

    check('경로까지 거리', dist, LIM_DIST, 'm')
    check('경로방향과 차이', hdiff, LIM_HEADING, '도')

    if node.cov is None:
        print('  AMCL 공분산 미수신 — AMCL 은 차가 움직일 때만 /amcl_pose 를 발행한다.')
        print('  조이패드로 차를 조금 움직인 뒤 다시 실행할 것.')
        ok = False
    else:
        check('AMCL 1시그마 x', math.sqrt(node.cov[0]), LIM_SIGMA_XY, 'm')
        check('AMCL 1시그마 y', math.sqrt(node.cov[7]), LIM_SIGMA_XY, 'm')
        check('AMCL 1시그마 yaw', math.degrees(math.sqrt(node.cov[35])), LIM_SIGMA_YAW, '도')

    print()
    if ok:
        print('>>> 출발 준비 완료')
    else:
        print('>>> 아직 미달. 헤어핀 곡선부에서 조이패드로 천천히 3~4m 주행해 볼 것.')
        print('    (긴 직선 회랑에서는 회랑을 따라가는 방향의 오차가 줄지 않는다)')

    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
