#!/usr/bin/env python3
"""맵 PGM 사본에 장애물을 찍는다. 좌표는 월드좌표(m).

  python3 tools/add_obstacles.py maps/big_0723.yaml maps/big_0723_obs \
      --box 10.0,7.0,0.4,0.4  --circle 8.0,5.0,0.15

--box    x,y,w,h  (x,y 는 중심)
--circle x,y,r
원본 맵은 건드리지 않고 <out>.pgm / <out>.yaml 을 새로 만든다.
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image


def to_rc(x, y, meta, height):
    """월드 (x,y) -> PGM 파일 기준 (row, col). PGM 은 위가 y 최대라 뒤집는다."""
    c = int((x - meta['origin'][0]) / meta['resolution'])
    r = height - 1 - int((y - meta['origin'][1]) / meta['resolution'])
    return r, c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src_yaml')
    ap.add_argument('out_prefix')
    ap.add_argument('--box', action='append', default=[], metavar='x,y,w,h')
    ap.add_argument('--circle', action='append', default=[], metavar='x,y,r')
    args = ap.parse_args()

    with open(args.src_yaml) as f:
        meta = yaml.safe_load(f)
    src_img = os.path.join(os.path.dirname(os.path.abspath(args.src_yaml)), meta['image'])
    img = np.array(Image.open(src_img))
    h, w = img.shape
    res = meta['resolution']

    rr, cc = np.mgrid[0:h, 0:w]
    for spec in args.box:
        x, y, bw, bh = [float(v) for v in spec.split(',')]
        r, c = to_rc(x, y, meta, h)
        dr, dc = int(round(bh / 2 / res)), int(round(bw / 2 / res))
        mask = (np.abs(rr - r) <= dr) & (np.abs(cc - c) <= dc)
        print(f'box    ({x}, {y}) {bw}x{bh}m -> row {r} col {c}, {mask.sum()} px')
        img[mask] = 0
    for spec in args.circle:
        x, y, rad = [float(v) for v in spec.split(',')]
        r, c = to_rc(x, y, meta, h)
        pr = rad / res
        mask = ((rr - r) ** 2 + (cc - c) ** 2) <= pr ** 2
        print(f'circle ({x}, {y}) r={rad}m -> row {r} col {c}, {mask.sum()} px')
        img[mask] = 0

    out_pgm = args.out_prefix + '.pgm'
    Image.fromarray(img).save(out_pgm)
    meta['image'] = os.path.basename(out_pgm)
    with open(args.out_prefix + '.yaml', 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    print(f'wrote {out_pgm} and {args.out_prefix}.yaml')


if __name__ == '__main__':
    main()
