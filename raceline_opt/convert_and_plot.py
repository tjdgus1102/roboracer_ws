#!/usr/bin/env python3
"""
Step 3: convert the TUM raceline to path_follower's (x,y,yaw,vx) CSV and plot it
over the map next to the centerline for visual verification.

Input : raceline_opt/traj_race_cl.csv  (s,x,y,psi,kappa,vx,ax) - closed
Output: raceline_opt/big_0723_raceline.csv  (# x,y,yaw,vx)  - open loop
        raceline_opt/big_0723_raceline_check.png

yaw is recomputed from consecutive points (REP-103: 0=+x, CCW) rather than
trusting TUM's psi convention. The path is written OPEN (start != goal) to
avoid the closed-loop goal-checker trap that makes Nav2 think it arrived at t=0.

The vx column is NOT the one in traj_race_cl.csv: that profile was computed
with racecar_f110.ini's fantasy limits (v_max 15 m/s, ggv 12 m/s^2 ~ 1.2g),
which peak at 7.5 m/s and demand -18 m/s^2 braking. The line geometry from
mincurv does not depend on speed, so we keep the geometry and recompute the
profile here with limits the real car can hold. Tune V_MAX / AX_MAX / AY_MAX.
"""
import math
import os

import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from trajectory_planning_helpers.calc_vel_profile import calc_vel_profile

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.expanduser("~/roboracer_ws")

# --- real-car limits for the velocity profile -------------------------------
V_MAX = 3.0     # [m/s]   must match FollowPath.vx_max in nav2_params_real.yaml
AX_MAX = 5.0    # [m/s^2] longitudinal grip (~0.5 g)
AY_MAX = 5.0    # [m/s^2] lateral grip (~0.5 g)
M_VEH = 3.518   # [kg]    from racecar_f110.ini
DRAG_COEFF = 0.0136


def _velocity_profile(kappa, el_lengths):
    """TUM forward/backward pass over the closed raceline with real limits."""
    ggv = np.array([[0.0, AX_MAX, AY_MAX], [V_MAX, AX_MAX, AY_MAX]])
    ax_max_machines = np.array([[0.0, AX_MAX], [V_MAX, AX_MAX]])
    vx = calc_vel_profile(
        ax_max_machines=ax_max_machines,
        kappa=kappa,
        el_lengths=el_lengths,
        closed=True,
        drag_coeff=DRAG_COEFF,
        m_veh=M_VEH,
        ggv=ggv,
        v_max=V_MAX,
        dyn_model_exp=1.0,
    )

    # tph's closed solver leaves a step at the loop seam (it stitches the
    # forward/backward passes at one point instead of relaxing around the
    # wrap), which asks for ~-7.5 m/s^2 there. Relax the braking constraint
    # around the loop until it holds everywhere. Diamond g-g: the longitudinal
    # grip left over is what the lateral load is not already using.
    for _ in range(3):
        for i in range(len(vx) - 1, -1, -1):
            j = (i + 1) % len(vx)
            ay = vx[i] ** 2 * abs(kappa[i])
            ax_avail = AX_MAX * max(0.0, 1.0 - ay / AY_MAX)
            vx[i] = min(vx[i], math.sqrt(vx[j] ** 2 + 2.0 * ax_avail * el_lengths[i]))
    return vx


def main():
    traj = np.loadtxt(os.path.join(HERE, "traj_race_cl.csv"), comments="#", delimiter=",")
    x, y = traj[:-1, 1], traj[:-1, 2]  # drop duplicate closing point
    vx = _velocity_profile(kappa=traj[:-1, 4], el_lengths=np.diff(traj[:, 0]))
    print(f"  profile: vx {vx.min():.2f}-{vx.max():.2f} m/s, "
          f"lap {np.sum(np.diff(traj[:, 0]) / vx):.2f} s "
          f"(limits v_max={V_MAX}, ax={AX_MAX}, ay={AY_MAX})")

    # Rotate the closed loop so the start/goal sits on the wide top straight
    # (w~2.9m, kappa~0.15) instead of the tight top-right notch (w~0.96m,
    # kappa~1.26) that index 0 originally landed on. Anchored to a coordinate,
    # not an index: the point count shifts whenever the widths are re-extracted.
    START_XY = (10.03, 7.74)
    start_idx = int(np.argmin(np.hypot(x - START_XY[0], y - START_XY[1])))
    print(f"  start anchored to {START_XY} -> index {start_idx}/{len(x)} "
          f"({x[start_idx]:.2f}, {y[start_idx]:.2f})")
    x = np.roll(x, -start_idx)
    y = np.roll(y, -start_idx)
    vx = np.roll(vx, -start_idx)

    # yaw from forward difference BEFORE trimming (uses the loop closure)
    yaw = np.arctan2(np.roll(y, -1) - y, np.roll(x, -1) - x)

    # Trim the tail so start != goal by ~0.7 m, matching the known-good
    # centerline gap. A near-closed loop trips the stateful goal checker into
    # "arrived at t=0". See memory: roboracer-closed-loop-goal-checker.
    GAP = 0.65
    n = len(x)
    while n > 2 and math.hypot(x[0] - x[n - 1], y[0] - y[n - 1]) < GAP:
        n -= 1
    x, y, yaw = x[:n], y[:n], yaw[:n]
    vx = vx[:n]
    print(f"  trimmed to {n} pts; start-goal gap "
          f"{math.hypot(x[0]-x[-1], y[0]-y[-1]):.2f} m")

    out_csv = os.path.join(HERE, "big_0723_raceline.csv")
    with open(out_csv, "w") as f:
        f.write("# x,y,yaw,vx\n")
        for xi, yi, yi_yaw, vi in zip(x, y, yaw, vx):
            f.write(f"{xi:.4f},{yi:.4f},{yi_yaw:.4f},{vi:.3f}\n")
    print(f"[ok] {len(x)} points -> {out_csv}")

    # --- verification plot ---
    with open(os.path.join(WS, "maps/big_0723.yaml")) as fh:
        m = yaml.safe_load(fh)
    grid = np.asarray(Image.open(os.path.join(WS, "maps", m["image"])).convert("L"))
    res = float(m["resolution"])
    ox, oy = float(m["origin"][0]), float(m["origin"][1])
    H, W = grid.shape
    extent = [ox, ox + W * res, oy, oy + H * res]

    cx, cy = [], []
    for line in open(os.path.join(WS, "paths/big_0723_center_open.csv")):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        a, b, _ = line.split(",")
        cx.append(float(a)); cy.append(float(b))

    plt.figure(figsize=(11, 8))
    plt.imshow(grid, cmap="gray", origin="upper", extent=extent)
    plt.plot(cx, cy, "b--", lw=1.0, alpha=0.6, label="centerline (current)")
    sc = plt.scatter(x, y, c=vx, cmap="jet", s=10, label="raceline (optimized)")
    plt.colorbar(sc, label=f"speed profile [m/s] (v_max={V_MAX}, {AX_MAX}/{AY_MAX} m/s2)")
    plt.plot(x[0], y[0], "ko", ms=8, label="start")
    plt.legend(loc="best", fontsize=8)
    plt.title("optimized raceline vs centerline  (verify it stays inside walls)")
    plt.axis("equal")
    png = os.path.join(HERE, "big_0723_raceline_check.png")
    plt.savefig(png, dpi=130, bbox_inches="tight")
    print(f"[ok] verification plot -> {png}")


if __name__ == "__main__":
    main()
