#!/usr/bin/env python3
"""
Acceptance test for the raceline: measure true wall clearance from the map.

Ray-cast track widths can lie (see extract_track_widths.py), so the only
trustworthy check is the distance transform of the occupancy grid evaluated on
the final path. Prints the clearance profile and writes a plot coloured by
clearance so tight spots are obvious.

Input : raceline_opt/big_0723_raceline.csv
Output: raceline_opt/big_0723_clearance.png
"""
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

CAR_HALF_WIDTH = 0.15   # racecar_f110.ini: width 0.30 m
MIN_CLEAR = 0.35        # what extract_track_widths.py was asked to guarantee

REGIONS = {"NE hairpin": lambda x, y: (x > 11.8) & (y > 6.5),
           "SW hairpin": lambda x, y: (x < 7.8) & (y < 2.3)}


def main():
    with open(os.path.join(WS, "maps/big_0723.yaml")) as fh:
        m = yaml.safe_load(fh)
    grid = np.asarray(Image.open(os.path.join(WS, "maps", m["image"])).convert("L"))
    res = float(m["resolution"])
    ox, oy = float(m["origin"][0]), float(m["origin"][1])
    H, W = grid.shape
    # only known-free (254) is drivable; unknown grey (205) is the island
    # interior and the outside of the room
    dist = distance_transform_edt(grid >= 254) * res

    d = np.loadtxt(os.path.join(HERE, "big_0723_raceline.csv"),
                   comments="#", delimiter=",")
    x, y = d[:, 0], d[:, 1]
    col = np.clip(((x - ox) / res).astype(int), 0, W - 1)
    row = np.clip((H - (y - oy) / res).astype(int), 0, H - 1)
    clear = dist[row, col]

    print(f"clearance over {len(x)} points: "
          f"min {clear.min():.3f}  mean {clear.mean():.3f}  max {clear.max():.3f} m")
    for name, sel in REGIONS.items():
        msk = sel(x, y)
        print(f"  {name}: {msk.sum():3d} pts, min {clear[msk].min():.3f} m")
    air = clear.min() - CAR_HALF_WIDTH
    print(f"  tightest point leaves {air:+.3f} m of air past the {CAR_HALF_WIDTH} m "
          f"car half-width")
    if clear.min() < CAR_HALF_WIDTH:
        print("  [FAIL] the car body intersects a wall")
    elif clear.min() < MIN_CLEAR - 0.05:
        print(f"  [WARN] below the {MIN_CLEAR} m target by more than 5 cm")
    else:
        print(f"  [OK] within 5 cm of the {MIN_CLEAR} m target")

    extent = [ox, ox + W * res, oy, oy + H * res]
    plt.figure(figsize=(11, 8))
    plt.imshow(grid, cmap="gray", origin="upper", extent=extent)
    sc = plt.scatter(x, y, c=clear, cmap="RdYlGn", s=22, vmin=0.15, vmax=1.0,
                     zorder=3, edgecolors="none")
    plt.colorbar(sc, label="wall clearance [m]")
    i = int(np.argmin(clear))
    plt.plot(x[i], y[i], "kx", ms=14, mew=2.5, zorder=4)
    plt.annotate(f"tightest {clear[i]:.2f} m\n({x[i]:.2f}, {y[i]:.2f})",
                 (x[i], y[i]), textcoords="offset points", xytext=(18, 18),
                 fontsize=9, fontweight="bold",
                 arrowprops=dict(arrowstyle="->", lw=1.2))
    plt.title(f"raceline wall clearance — min {clear.min():.3f} m "
              f"(car half-width {CAR_HALF_WIDTH} m)")
    plt.xlabel("x [m]"); plt.ylabel("y [m]")
    plt.axis("equal")
    png = os.path.join(HERE, "big_0723_clearance.png")
    plt.savefig(png, dpi=130, bbox_inches="tight")
    print(f"[ok] {png}")


if __name__ == "__main__":
    main()
