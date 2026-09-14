#!/usr/bin/env python3
"""
Plot the raceline over the map with START / GOAL clearly marked.

Input : raceline_opt/big_0723_raceline.csv  (# x,y,yaw)
Output: raceline_opt/big_0723_raceline_start_goal.png

The start/goal cut sits on the wide top straight (corridor ~2.9 m) rather than
the tight top-right notch (~0.96 m); an inset zooms in on that region so the
free space around both points is visible.
"""
import math
import os

import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.expanduser("~/roboracer_ws")

XY_GOAL_TOL = 0.3  # Nav2 xy_goal_tolerance, drawn as the goal acceptance circle


def load_path(path):
    x, y, yaw = [], [], []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        a, b, c = line.split(",")
        x.append(float(a)); y.append(float(b)); yaw.append(float(c))
    return np.array(x), np.array(y), np.array(yaw)


def draw(ax, grid, extent, x, y, yaw, arrow_every):
    ax.imshow(grid, cmap="gray", origin="upper", extent=extent)
    ax.plot(np.append(x, x[0]), np.append(y, y[0]), "-", color="steelblue", lw=1.4)
    for i in range(0, len(x) - 1, arrow_every):
        ax.annotate("", xy=(x[i + 1], y[i + 1]), xytext=(x[i], y[i]),
                    arrowprops=dict(arrowstyle="-|>", color="steelblue", lw=1.1))

    ax.add_patch(plt.Circle((x[-1], y[-1]), XY_GOAL_TOL, color="red",
                            fill=False, ls="--", lw=1.2))
    ax.plot([x[0], x[-1]], [y[0], y[-1]], ":", color="darkorange", lw=1.4)

    for xi, yi, yi_yaw, col in ((x[0], y[0], yaw[0], "green"),
                                (x[-1], y[-1], yaw[-1], "red")):
        ax.arrow(xi, yi, 0.55 * math.cos(yi_yaw), 0.55 * math.sin(yi_yaw),
                 color=col, width=0.04, head_width=0.18, length_includes_head=True,
                 zorder=5)


def main():
    x, y, yaw = load_path(os.path.join(HERE, "big_0723_raceline.csv"))

    with open(os.path.join(WS, "maps/big_0723.yaml")) as fh:
        m = yaml.safe_load(fh)
    grid = np.asarray(Image.open(os.path.join(WS, "maps", m["image"])).convert("L"))
    res = float(m["resolution"])
    ox, oy = float(m["origin"][0]), float(m["origin"][1])
    H, W = grid.shape
    extent = [ox, ox + W * res, oy, oy + H * res]

    length = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
    gap = math.hypot(x[0] - x[-1], y[0] - y[-1])

    fig, (ax, axz) = plt.subplots(1, 2, figsize=(16, 8),
                                  gridspec_kw={"width_ratios": [1.55, 1]})

    draw(ax, grid, extent, x, y, yaw, arrow_every=8)
    ax.plot(x[0], y[0], "o", color="green", ms=13, mec="k", zorder=6,
            label=f"START  ({x[0]:.2f}, {y[0]:.2f})  yaw {math.degrees(yaw[0]):.0f}°")
    ax.plot(x[-1], y[-1], "*", color="red", ms=20, mec="k", zorder=6,
            label=f"GOAL   ({x[-1]:.2f}, {y[-1]:.2f})  yaw {math.degrees(yaw[-1]):.0f}°")
    ax.plot([], [], "-", color="steelblue", lw=1.4,
            label=f"raceline  {len(x)} pts, {length:.2f} m")
    ax.legend(loc="lower left", fontsize=9, framealpha=0.92)
    ax.set_xlabel("x [m] (map frame)")
    ax.set_ylabel("y [m] (map frame)")
    ax.set_title("big_0723_raceline — START / GOAL\narrows = driving direction")
    ax.set_aspect("equal")

    # --- inset: zoom on the start/goal region ---
    pad = 1.9
    cx, cy = (x[0] + x[-1]) / 2, (y[0] + y[-1]) / 2
    draw(axz, grid, extent, x, y, yaw, arrow_every=3)
    axz.plot(x[0], y[0], "o", color="green", ms=15, mec="k", zorder=6)
    axz.plot(x[-1], y[-1], "*", color="red", ms=24, mec="k", zorder=6)
    axz.annotate(f"START\n({x[0]:.2f}, {y[0]:.2f})", (x[0], y[0]),
                 textcoords="offset points", xytext=(-12, -38), ha="center",
                 fontsize=10, fontweight="bold", color="darkgreen")
    axz.annotate(f"GOAL\n({x[-1]:.2f}, {y[-1]:.2f})", (x[-1], y[-1]),
                 textcoords="offset points", xytext=(14, 26), ha="center",
                 fontsize=10, fontweight="bold", color="darkred")
    axz.annotate(f"start–goal {gap:.2f} m", (cx, cy),
                 textcoords="offset points", xytext=(0, -14), ha="center",
                 fontsize=9, fontweight="bold", color="darkorange")
    axz.annotate(f"xy_goal_tolerance {XY_GOAL_TOL} m",
                 (x[-1], y[-1] - XY_GOAL_TOL), textcoords="offset points",
                 xytext=(0, -12), ha="center", fontsize=8, color="red")
    axz.set_xlim(cx - pad, cx + pad)
    axz.set_ylim(cy - pad, cy + pad)
    axz.set_xlabel("x [m]")
    axz.set_title("zoom: wide straight (corridor ~2.9 m)")
    axz.set_aspect("equal")

    png = os.path.join(HERE, "big_0723_raceline_start_goal.png")
    fig.savefig(png, dpi=130, bbox_inches="tight")
    print(f"[ok] {png}")
    print(f"     START ({x[0]:.2f}, {y[0]:.2f}) yaw {math.degrees(yaw[0]):.1f}°")
    print(f"     GOAL  ({x[-1]:.2f}, {y[-1]:.2f}) yaw {math.degrees(yaw[-1]):.1f}°")
    print(f"     gap {gap:.2f} m, {len(x)} pts, {length:.2f} m")


if __name__ == "__main__":
    main()
