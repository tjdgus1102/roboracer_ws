#!/usr/bin/env python3
"""
Step 2: run the TUM/ForzaETH minimum-curvature raceline optimizer on big_0723.

Builds a self-contained input directory (F1TENTH params + our width CSV), calls
trajectory_optimizer(), and writes the result to raceline_opt/traj_race_cl.csv.
Nothing under the teammate's src/ is modified.

Returned trajectory columns (TUM convention):
    s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
"""
import os
import shutil
import sys
import traceback

import numpy as np

# tph 0.76 was written for old scipy; new scipy's spatial.distance.euclidean
# strictly requires 1-D inputs while tph passes splev outputs shaped (2,1).
# Wrap it to ravel the args (runtime shim only; no library file is edited).
import scipy.spatial.distance as _sdist
_orig_euclid = _sdist.euclidean
def _euclid_safe(u, v, w=None):
    return _orig_euclid(np.ravel(u), np.ravel(v), w)
_sdist.euclidean = _euclid_safe

HERE = os.path.dirname(os.path.abspath(__file__))
RS = "/home/swlee/F1TENTH/race_stack"
TUM = os.path.join(
    RS, "planner/global_planner/global_planner/"
    "global_racetrajectory_optimization/global_racetrajectory_optimization")
GP_CFG = os.path.join(RS, "stack_master/config/global_planner")

TRACK = "big_0723"
# tph uses dev_max = w_tr - safety_width/2. Since extract_track_widths.py now
# caps every ray at min_clear (0.30 m = car half-width + margin), the widths it
# writes are already the safe corridor and the margin must not be counted twice.
SAFETY_WIDTH = 0.0
CURV_OPT = "mincurv"      # smoothest line; softest constraints (safest first try)


def build_input_dir():
    run_dir = os.path.join(HERE, "tum_run")
    shutil.rmtree(run_dir, ignore_errors=True)
    os.makedirs(os.path.join(run_dir, "veh_dyn_info"))
    os.makedirs(os.path.join(run_dir, "frictionmaps"), exist_ok=True)
    # F1TENTH vehicle params + dynamics
    shutil.copy(os.path.join(GP_CFG, "racecar_f110.ini"),
                os.path.join(run_dir, "racecar_f110.ini"))
    for f in ("ggv.csv", "ax_max_machines.csv"):
        shutil.copy(os.path.join(GP_CFG, "veh_dyn_info", f),
                    os.path.join(run_dir, "veh_dyn_info", f))
    # our track (centerline + widths)
    shutil.copy(os.path.join(HERE, f"{TRACK}_tum_input.csv"),
                os.path.join(run_dir, f"{TRACK}.csv"))
    return run_dir


def main():
    sys.path.insert(0, os.path.dirname(TUM))  # parent, so the package is importable
    from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer

    run_dir = build_input_dir()
    cwd0 = os.getcwd()
    os.chdir(run_dir)  # track_file is opened relative to CWD
    try:
        traj, bound1, bound2, laptime = trajectory_optimizer(
            input_path=run_dir, track_name=TRACK,
            curv_opt_type=CURV_OPT, safety_width=SAFETY_WIDTH, plot=False)
    except Exception:
        traceback.print_exc()
        os.chdir(cwd0)
        sys.exit(1)
    os.chdir(cwd0)

    traj = np.asarray(traj)
    out = os.path.join(HERE, "traj_race_cl.csv")
    np.savetxt(out, traj, delimiter=",",
               header="s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2", comments="# ")

    kappa = np.abs(traj[:, 4])
    kmax = kappa.max()
    rmin = 1.0 / kmax if kmax > 1e-6 else float("inf")
    vx = traj[:, 5]
    print(f"[ok] {len(traj)} points -> {out}")
    print(f"  estimated lap time : {laptime:.2f} s")
    print(f"  speed  min/max     : {vx.min():.2f} / {vx.max():.2f} m/s")
    print(f"  max curvature      : {kmax:.3f} rad/m  -> min radius {rmin:.3f} m")
    car_rmin = 0.69
    if rmin < car_rmin:
        print(f"  [WARN] min radius {rmin:.2f} m < car limit {car_rmin} m "
              f"-> car may NOT be able to follow this line (tighten curvlim / widen safety_width)")
    else:
        print(f"  [OK] min radius {rmin:.2f} m >= car limit {car_rmin} m -> feasible")
    # boundaries for the plot in step 3
    np.savetxt(os.path.join(HERE, "bound_inner.csv"), bound1, delimiter=",")
    np.savetxt(os.path.join(HERE, "bound_outer.csv"), bound2, delimiter=",")


if __name__ == "__main__":
    main()
