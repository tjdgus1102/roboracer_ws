#!/usr/bin/env python3
"""중심선(medial axis)을 곡률 제한 주행 경로로 다듬는다.

medial axis는 벽에서 등거리인 선이라 방 모서리를 바짝 끼고 돌아 곡률반경이 작다.
차량 최소 회전반경보다 작은 코너는 아무리 천천히 가도 물리적으로 못 돈다.
회랑은 그보다 넓으므로 라인을 완만하게 잡을 여지가 있다.

정식화(최소곡률 레이싱라인): 중심선의 각 점에서 법선 방향 횡오프셋 a_i 만 변수로 두고
  minimize  sum |p_{i-1} - 2 p_i + p_{i+1}|^2,   p_i = ref_i + a_i * n_i
  s.t.      |a_i| <= (벽까지 거리) - clearance
법선 위로만 움직이므로 루프가 수축하지 않는다. 투영 경사하강으로 푼다.

  python3 smooth_path.py <map.yaml> <in.csv> <out.csv> [--clearance 0.25] [--min-radius 0.85]
"""
import argparse

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage


def load_map(path):
    with open(path) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(path.rsplit('/', 1)[0] + '/' + meta['image']).convert('L'))
    res = meta['resolution']
    free = ((255.0 - img.astype(np.float32)) / 255.0) < meta.get('free_thresh', 0.196) * 0.5
    dist = ndimage.distance_transform_edt(free) * res      # 벽까지 거리(m)
    return dist, res, meta['origin'][0], meta['origin'][1], img.shape[0]


def sample(field, xy, res, ox, oy, h):
    c = np.clip((xy[:, 0] - ox) / res, 0, field.shape[1] - 1.001)
    r = np.clip(h - 1 - (xy[:, 1] - oy) / res, 0, field.shape[0] - 1.001)
    r0, c0 = np.floor(r).astype(int), np.floor(c).astype(int)
    dr, dc = r - r0, c - c0
    return (field[r0, c0] * (1 - dr) * (1 - dc) + field[r0 + 1, c0] * dr * (1 - dc) +
            field[r0, c0 + 1] * (1 - dr) * dc + field[r0 + 1, c0 + 1] * dr * dc)


def radii(xy, k=3):
    n = len(xy)
    a, b, c = xy[np.arange(n) - k], xy, xy[(np.arange(n) + k) % n]
    A = np.linalg.norm(b - c, axis=1)
    B = np.linalg.norm(a - c, axis=1)
    C = np.linalg.norm(a - b, axis=1)
    s = np.abs(np.cross(b - a, c - a)) / 2
    return np.where(s > 1e-9, A * B * C / (4 * np.maximum(s, 1e-9)), 1e9)


def resample(xy, step):
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]), axis=0), axis=1))]
    t = np.linspace(0, d[-1], max(int(d[-1] / step), 10), endpoint=False)
    return np.c_[np.interp(t, d, np.r_[xy[:, 0], xy[0, 0]]),
                 np.interp(t, d, np.r_[xy[:, 1], xy[0, 1]])]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_yaml')
    ap.add_argument('in_csv')
    ap.add_argument('out_csv')
    ap.add_argument('--clearance', type=float, default=0.25, help='벽에서 유지할 최소 거리(m)')
    ap.add_argument('--min-radius', type=float, default=0.85, help='목표 최소 곡률반경(m)')
    ap.add_argument('--iters', type=int, default=8000)
    ap.add_argument('--lr', type=float, default=0.05)
    args = ap.parse_args()

    dist, res, ox, oy, h = load_map(args.map_yaml)
    ref = resample(np.loadtxt(args.in_csv, delimiter=',', comments='#')[:, :2], 0.10)
    n = len(ref)

    # 중심선 접선 -> 법선
    tang = np.roll(ref, -1, 0) - np.roll(ref, 1, 0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    nrm = np.c_[-tang[:, 1], tang[:, 0]]

    # 횡오프셋 상한: 그 지점에서 벽까지 거리 - 여유
    bound = np.maximum(sample(dist, ref, res, ox, oy, h) - args.clearance, 0.0)
    print(f'입력 중심선: {n}점, 최소 곡률반경 {radii(ref).min():.2f}m')
    print(f'횡방향 여유: 중앙 ±{np.median(bound):.2f}m, 최소 ±{bound.min():.2f}m')

    # 등간격 ds에서 2차차분 크기 |D| ~= ds^2 / R  ->  목표 R 이상이려면 |D| <= ds^2/R_target
    ds = lap0 = np.linalg.norm(np.diff(np.vstack([ref, ref[:1]]), axis=0), axis=1).mean()
    Dmax = ds ** 2 / args.min_radius

    a = np.zeros(n)
    for _ in range(args.iters):
        p = ref + a[:, None] * nrm
        D = np.roll(p, 1, 0) - 2 * p + np.roll(p, -1, 0)          # 2차 차분
        mag = np.linalg.norm(D, axis=1) + 1e-12
        hp = 2 * np.maximum(mag - Dmax, 0.0)                        # 힌지: 목표 초과분만 벌점
        u = (hp / mag)[:, None] * D                                # dE/dD_k
        g = ((np.roll(u, -1, 0) * nrm).sum(1)
             - 2 * (u * nrm).sum(1)
             + (np.roll(u, 1, 0) * nrm).sum(1))
        a = np.clip(a - args.lr * g, -bound, bound)                # 투영: 회랑 밖 금지

    p = ref + a[:, None] * nrm
    R = radii(p)
    d = sample(dist, p, res, ox, oy, h)
    lap = np.sum(np.linalg.norm(np.diff(np.vstack([p, p[:1]]), axis=0), axis=1))
    yaw = np.arctan2(np.roll(p[:, 1], -1) - p[:, 1], np.roll(p[:, 0], -1) - p[:, 0])
    np.savetxt(args.out_csv, np.c_[p, yaw], delimiter=',', fmt='%.4f', header='x,y,yaw')

    ok = R.min() >= args.min_radius and d.min() >= args.clearance - 0.03
    print(f'출력 경로: {n}점, 랩 {lap:.1f}m')
    print(f'  최소 곡률반경 {R.min():.2f}m  (목표 {args.min_radius}m, 차량 한계 0.74m)')
    print(f'  벽까지 최소 여유 {d.min():.2f}m  (목표 {args.clearance}m)')
    print(f'  판정: {"통과" if ok else "미달"}')
    print(f'저장: {args.out_csv}')


if __name__ == '__main__':
    main()
