#!/usr/bin/env python3
"""
Step 1 of raceline optimization: build the TUM main_globaltraj.py input CSV.

Reads a ROS occupancy-grid map (.pgm + .yaml) and a centerline CSV (x,y,yaw)
and, for each centerline point, casts a ray to the left and right walls to
measure the drivable half-widths. Output format (TUM reftrack):

    # x_m,y_m,w_tr_right_m,w_tr_left_m

A ray also stops as soon as the wall clearance along it drops below
--min-clear. Without that cap, a ray whose normal happens to point *along* a
narrow corridor (the NE hairpin around the island tip, where the centerline
tangent rotates ~20 deg per station) slides down the corridor instead of
crossing it and reports 2.78 m where only 0.80 m exists -- which is what let
the optimizer cut that corner to 0.07 m from the wall. Stopping at the FIRST
violation means every offset in [0, w] is guaranteed >= min_clear from a wall,
so min_clear carries the whole safety margin and TUM runs with safety_width=0.

Also writes a debug overlay PNG so the widths can be eyeballed before feeding
TUM (coordinate-transform mistakes are the usual failure here).

Nothing in the teammate's workspace is touched: this script lives under
~/roboracer_ws/raceline_opt/ and only reads maps/ and paths/.
"""
import argparse
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.expanduser("~/roboracer_ws")


def load_map(yaml_path):
    with open(yaml_path) as f:
        m = yaml.safe_load(f)
    img_path = m["image"]
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    grid = np.asarray(Image.open(img_path).convert("L"))  # (H, W), row 0 = top
    res = float(m["resolution"])
    ox, oy = float(m["origin"][0]), float(m["origin"][1])
    occ_thresh = float(m.get("occupied_thresh", 0.65))
    # trinary, negate=0: p_occ = (255 - pixel)/255 ; occupied if p_occ > occ_thresh
    wall_pixel = 255.0 * (1.0 - occ_thresh)
    return grid, res, ox, oy, wall_pixel


def load_centerline(csv_path):
    pts = []
    for line in open(csv_path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        x, y, _yaw = (float(v) for v in line.split(","))
        pts.append((x, y))
    return np.array(pts)  # (N, 2)


def resample_closed(pts, spacing):
    """Resample the closed centerline to ~`spacing` m between stations.

    TUM fits a spline through the stations it is given; at 0.24 m spacing that
    fit deviates up to 0.11 m around the NE hairpin, which comes straight out
    of the clearance budget. Denser stations pin the spline down. The teammate's
    paths/big_0723_center_open.csv is never modified -- this is in-memory only.
    """
    loop = np.vstack([pts, pts[:1]])
    seg = np.hypot(*np.diff(loop, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(int(round(s[-1] / spacing)), len(pts))
    s_new = np.linspace(0.0, s[-1], n, endpoint=False)
    return np.column_stack([np.interp(s_new, s, loop[:, 0]),
                            np.interp(s_new, s, loop[:, 1])])


class MapQuery:
    def __init__(self, grid, res, ox, oy, wall_pixel, min_clear):
        self.grid = grid
        self.H, self.W = grid.shape
        self.res = res
        self.ox, self.oy = ox, oy
        self.wall_pixel = wall_pixel
        self.min_clear = min_clear
        # Distance to the nearest non-free cell, in metres. Only known-free
        # (254) counts as free: unknown grey (205) is the island interior and
        # the outside of the room, neither of which is drivable.
        self.clear_m = distance_transform_edt(grid >= 254) * res

    def world_to_pixel(self, wx, wy):
        col = int(round((wx - self.ox) / self.res))
        row = int(round(self.H - 1 - (wy - self.oy) / self.res))
        return row, col

    def is_wall(self, wx, wy):
        row, col = self.world_to_pixel(wx, wy)
        if row < 0 or row >= self.H or col < 0 or col >= self.W:
            return True  # off the map counts as a wall (stop the ray)
        return self.grid[row, col] < self.wall_pixel

    def clearance(self, wx, wy):
        row, col = self.world_to_pixel(wx, wy)
        if row < 0 or row >= self.H or col < 0 or col >= self.W:
            return 0.0
        return float(self.clear_m[row, col])

    def ray_to_wall(self, x, y, nx, ny, max_dist, step):
        """March from (x,y) along unit normal (nx,ny) until a wall or until the
        clearance drops below min_clear; return the last safe distance."""
        d = step
        while d <= max_dist:
            px, py = x + nx * d, y + ny * d
            if self.is_wall(px, py) or self.clearance(px, py) < self.min_clear:
                return d - step
            d += step
        return max_dist  # no wall found within max_dist


def tangents(pts, closed=True):
    n = len(pts)
    tang = np.zeros_like(pts)
    for i in range(n):
        if closed:
            p_prev, p_next = pts[(i - 1) % n], pts[(i + 1) % n]
        else:
            p_prev, p_next = pts[max(i - 1, 0)], pts[min(i + 1, n - 1)]
        t = p_next - p_prev
        norm = math.hypot(t[0], t[1])
        tang[i] = t / norm if norm > 1e-9 else (1.0, 0.0)
    return tang


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=os.path.join(WS, "maps/big_0723.yaml"))
    ap.add_argument("--centerline", default=os.path.join(WS, "paths/big_0723_center_open.csv"))
    ap.add_argument("--out", default=os.path.join(HERE, "big_0723_tum_input.csv"))
    ap.add_argument("--debug-png", default=os.path.join(HERE, "big_0723_widths_debug.png"))
    ap.add_argument("--max-dist", type=float, default=2.5, help="max ray length (m)")
    ap.add_argument("--step", type=float, default=0.01, help="ray march step (m)")
    ap.add_argument("--resample", type=float, default=0.10,
                    help="station spacing (m) for the reference line; 0 keeps the input points")
    ap.add_argument("--min-clear", type=float, default=0.35,
                    help="required wall clearance (m): car half-width 0.15 + margin. "
                         "The whole safety margin lives here, so run TUM with safety_width=0. "
                         "0.35 is near the track's ceiling -- 0.38 aborts at the SW hairpin, "
                         "where the centerline itself only clears 0.40 m")
    args = ap.parse_args()

    grid, res, ox, oy, wall_pixel = load_map(args.map)
    q = MapQuery(grid, res, ox, oy, wall_pixel, args.min_clear)
    pts = load_centerline(args.centerline)
    if args.resample > 0:
        n0 = len(pts)
        pts = resample_closed(pts, args.resample)
        print(f"  resampled centerline {n0} -> {len(pts)} stations "
              f"(~{args.resample} m spacing)")
    tang = tangents(pts, closed=True)

    rows = []
    left_hits, right_hits = [], []
    for (x, y), (tx, ty) in zip(pts, tang):
        # right of travel = (ty, -tx), left of travel = (-ty, tx)
        rnx, rny = ty, -tx
        lnx, lny = -ty, tx
        w_right = q.ray_to_wall(x, y, rnx, rny, args.max_dist, args.step)
        w_left = q.ray_to_wall(x, y, lnx, lny, args.max_dist, args.step)
        rows.append((x, y, w_right, w_left))
        right_hits.append((x + rnx * w_right, y + rny * w_right))
        left_hits.append((x + lnx * w_left, y + lny * w_left))

    # A non-positive half-width means the centerline itself is closer to a wall
    # than min_clear, which makes TUM's dev_max bounds degenerate. Catch it here
    # rather than inside the QP.
    bad = [(i, r) for i, r in enumerate(rows) if r[2] <= 0.0 or r[3] <= 0.0]
    if bad:
        for i, (x, y, wr, wl) in bad:
            print(f"  [FAIL] point {i} ({x:.2f},{y:.2f}) wr={wr:.2f} wl={wl:.2f} "
                  f"clearance={q.clearance(x, y):.2f} < min_clear={args.min_clear}")
        raise SystemExit(f"[abort] {len(bad)} centerline points are within "
                         f"{args.min_clear} m of a wall -- lower --min-clear")

    with open(args.out, "w") as f:
        f.write("# x_m,y_m,w_tr_right_m,w_tr_left_m\n")
        for x, y, wr, wl in rows:
            f.write(f"{x:.4f},{y:.4f},{wr:.4f},{wl:.4f}\n")

    widths = np.array([[r[2], r[3]] for r in rows])
    total = widths[:, 0] + widths[:, 1]
    print(f"[ok] {len(rows)} points -> {args.out}")
    print(f"  right half-width min/mean/max: "
          f"{widths[:,0].min():.2f}/{widths[:,0].mean():.2f}/{widths[:,0].max():.2f} m")
    print(f"  left  half-width min/mean/max: "
          f"{widths[:,1].min():.2f}/{widths[:,1].mean():.2f}/{widths[:,1].max():.2f} m")
    print(f"  total track width min/mean/max: "
          f"{total.min():.2f}/{total.mean():.2f}/{total.max():.2f} m")
    maxed = np.sum((widths >= args.max_dist - args.step))
    if maxed:
        print(f"  [warn] {maxed} rays hit no wall within {args.max_dist} m "
              f"(open boundary or gap) -> check the debug PNG")

    # debug overlay
    left_hits = np.array(left_hits)
    right_hits = np.array(right_hits)
    extent = [ox, ox + q.W * res, oy, oy + q.H * res]
    plt.figure(figsize=(10, 8))
    plt.imshow(grid, cmap="gray", origin="upper", extent=extent)
    plt.plot(pts[:, 0], pts[:, 1], "b.-", ms=3, lw=0.8, label="centerline")
    plt.plot(left_hits[:, 0], left_hits[:, 1], "g.", ms=4, label="left wall")
    plt.plot(right_hits[:, 0], right_hits[:, 1], "r.", ms=4, label="right wall")
    plt.legend(loc="best", fontsize=8)
    plt.title("track width extraction (verify walls sit on the real walls)")
    plt.axis("equal")
    plt.savefig(args.debug_png, dpi=130, bbox_inches="tight")
    print(f"[ok] debug overlay -> {args.debug_png}")


if __name__ == "__main__":
    main()
