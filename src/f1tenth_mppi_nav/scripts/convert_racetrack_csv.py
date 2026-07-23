#!/usr/bin/env python3
"""Convert an f1tenth_racetracks *_centerline.csv (x_m, y_m, w_tr_right_m,
w_tr_left_m) into the x,y,yaw CSV format that path_follower.py expects.

Usage:
    python3 convert_racetrack_csv.py <input_centerline.csv> <output.csv>
"""
import argparse
import csv
import math


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_csv')
    parser.add_argument('output_csv')
    args = parser.parse_args()

    pts = []
    with open(args.input_csv) as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith('#'):
                continue
            x, y = float(row[0]), float(row[1])
            pts.append((x, y))

    n = len(pts)
    with open(args.output_csv, 'w') as f:
        f.write('# x,y,yaw\n')
        for i, (x, y) in enumerate(pts):
            nx_, ny_ = pts[(i + 1) % n]
            yaw = math.atan2(ny_ - y, nx_ - x)
            f.write(f'{x:.4f},{y:.4f},{yaw:.4f}\n')

    print(f'Wrote {n} waypoints to {args.output_csv}')


if __name__ == '__main__':
    main()
