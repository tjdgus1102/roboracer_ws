#!/usr/bin/env python3
"""기록한 경로 CSV 가 MPPI 로 주행 가능한지 확인한다.

    python3 check_path.py paths/track_real_lapN.csv

최소 곡률반경이 실차 최소 회전반경(0.927 m) 이상이어야 한다.
시작-끝 거리가 xy_goal_tolerance(0.3 m) 보다 가까우면 출발 즉시 도착 판정이 난다.
"""
import sys

import numpy as np

MIN_TURN_R = 0.927   # 실차 최소 회전반경 [m]
GOAL_TOL = 0.30      # xy_goal_tolerance [m]


def radii(xy):
    out = []
    for i in range(1, len(xy) - 1):
        a, b, c = xy[i - 1], xy[i], xy[i + 1]
        A = np.linalg.norm(b - c)
        B = np.linalg.norm(a - c)
        C = np.linalg.norm(a - b)
        s = abs(np.cross(b - a, c - a)) / 2
        out.append(A * B * C / (4 * s) if s > 1e-9 else np.inf)
    return np.array(out)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    data = np.loadtxt(sys.argv[1], delimiter=',', comments='#')
    if data.ndim != 2 or len(data) < 3:
        print('경로가 비어 있거나 점이 너무 적습니다. (record_path 가 죽었을 수 있음)')
        return 1

    xy = data[:, :2]
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    gap = float(np.linalg.norm(xy[0] - xy[-1]))
    R = radii(xy)
    bad = int((R < MIN_TURN_R).sum())

    print('점 %d개,  길이 %.2f m,  점 간격 최소 %.3f m' % (len(xy), seg.sum(), seg.min()))
    print()
    print('최소 곡률반경   %.3f m   (실차 한계 %.3f m)' % (R.min(), MIN_TURN_R))
    print('위반 구간       %d / %d' % (bad, len(R)))
    print('시작-끝 거리    %.3f m   %s'
          % (gap, 'OK' if gap > GOAL_TOL + 0.05 else 'X  <- 출발 즉시 도착 판정 위험'))

    if bad:
        print()
        print('가장 조인 지점 5곳:')
        for i in np.argsort(R)[:5]:
            print('   (%.2f, %.2f)   R=%.3f m' % (xy[i + 1][0], xy[i + 1][1], R[i]))

    print()
    if bad == 0 and gap > GOAL_TOL + 0.05:
        print('>>> 주행 가능')
    else:
        print('>>> 위반 있음. 헤어핀에서 벽에 붙지 말고 크게 돌아 다시 기록할 것.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
