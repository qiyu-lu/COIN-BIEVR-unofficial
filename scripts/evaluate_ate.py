#!/usr/bin/env python3
"""Absolute Trajectory Error (ATE RMSE) between an estimated TUM trajectory and a ground truth.

The estimate is the IMU pose logged by the pipeline (`debug.trajectory_path`). Ground truths
recorded with a total station only observe the position of a prism, so an optional lever arm
(prism position expressed in the IMU frame) is applied to the estimate before the comparison.

The two trajectories are associated in time by linearly interpolating the higher-rate one at the
stamps of the lower-rate one, aligned with a rigid SE(3) Umeyama fit (no scale) and compared by
their position error.
"""
import argparse
import json
import sys

import numpy as np


def load_tum(path):
    """Loads `t x y z qx qy qz qw` rows (whitespace or comma separated, '#' comments)."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 4:
                continue
            try:
                vals = [float(p) for p in parts[:8]]
            except ValueError:
                continue
            vals += [0.0] * (8 - len(vals))
            rows.append(vals)
    data = np.asarray(rows, dtype=np.float64)
    if data.size == 0:
        raise ValueError("no poses found in %s" % path)
    order = np.argsort(data[:, 0], kind="stable")
    data = data[order]
    keep = np.concatenate(([True], np.diff(data[:, 0]) > 0))
    return data[keep]


def quat_to_rot(q):
    """(N,4) quaternions as qx qy qz qw -> (N,3,3) rotation matrices."""
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def apply_lever_arm(est, lever_arm):
    """Position of a point rigidly attached to the body: p + R * lever_arm."""
    if lever_arm is None or not np.any(lever_arm):
        return est[:, 1:4].copy()
    R = quat_to_rot(est[:, 4:8])
    return est[:, 1:4] + R @ np.asarray(lever_arm, dtype=np.float64)


def interpolate(t_src, p_src, t_query, max_gap):
    """Linear interpolation of positions; queries inside gaps larger than max_gap are dropped."""
    idx = np.searchsorted(t_src, t_query, side="right")
    valid = (idx > 0) & (idx < len(t_src))
    idx = np.clip(idx, 1, len(t_src) - 1)
    t0, t1 = t_src[idx - 1], t_src[idx]
    valid &= (t1 - t0) <= max_gap
    alpha = np.where(valid, (t_query - t0) / np.maximum(t1 - t0, 1e-12), 0.0)
    p = p_src[idx - 1] + alpha[:, None] * (p_src[idx] - p_src[idx - 1])
    return p, valid


def umeyama(src, dst):
    """Rigid transform (R, t) minimizing sum ||R src + t - dst||^2."""
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    t = mu_d - R @ mu_s
    return R, t


def estimate_time_offset(est, gt, lever_arm=None, max_gap=1.0, search=5.0):
    """Offset (added to the estimate stamps) that minimizes the ATE, by coarse-to-fine search.

    Needed for ground truths whose clock is not synchronized with the sensors (the error is then
    proportional to the velocity and vanishes at standstill).
    """
    best, radius, step = 0.0, search, 0.1
    while step >= 1e-3:
        candidates = best + np.arange(-radius, radius + 0.5 * step, step)
        errors = []
        for offset in candidates:
            try:
                errors.append(evaluate_arrays(est, gt, lever_arm, max_gap, offset)["rmse"])
            except ValueError:
                errors.append(np.inf)
        best = float(candidates[int(np.argmin(errors))])
        radius, step = step, step / 10.0
    return best


def evaluate(est_path, gt_path, lever_arm=None, max_gap=1.0, time_offset=0.0):
    """`time_offset` is a fixed offset in seconds or "auto" to estimate it."""
    est = load_tum(est_path)
    gt = load_tum(gt_path)
    if time_offset == "auto":
        time_offset = estimate_time_offset(est, gt, lever_arm, max_gap)
    return evaluate_arrays(est, gt, lever_arm, max_gap, float(time_offset))


def evaluate_arrays(est, gt, lever_arm=None, max_gap=1.0, time_offset=0.0):
    est = est.copy()
    est[:, 0] += time_offset
    p_est_all = apply_lever_arm(est, lever_arm)

    dt_est = np.median(np.diff(est[:, 0])) if len(est) > 1 else np.inf
    dt_gt = np.median(np.diff(gt[:, 0])) if len(gt) > 1 else np.inf
    if dt_gt > dt_est:
        # Sparse ground truth: evaluate at its stamps, interpolating the estimate.
        t = gt[:, 0]
        p_est, valid = interpolate(est[:, 0], p_est_all, t, max_gap)
        p_gt = gt[:, 1:4]
    else:
        t = est[:, 0]
        p_gt, valid = interpolate(gt[:, 0], gt[:, 1:4], t, max_gap)
        p_est = p_est_all
    t, p_est, p_gt = t[valid], p_est[valid], p_gt[valid]
    if len(t) < 3:
        raise ValueError("only %d associated poses" % len(t))

    R, trans = umeyama(p_est, p_gt)
    p_aligned = p_est @ R.T + trans
    err = np.linalg.norm(p_aligned - p_gt, axis=1)
    length = float(np.sum(np.linalg.norm(np.diff(p_gt, axis=0), axis=1)))
    rmse = float(np.sqrt(np.mean(err ** 2)))
    return {
        "rmse": rmse,
        "mean": float(err.mean()),
        "median": float(np.median(err)),
        "max": float(err.max()),
        "n_pairs": int(len(t)),
        "gt_length": length,
        "rmse_percent": 100.0 * rmse / length if length > 0 else float("nan"),
        "coverage": float((t[-1] - t[0]) / max(gt[-1, 0] - gt[0, 0], 1e-9)),
        "final_error": float(err[-1]),
        "time_offset": float(time_offset),
        "_t": t,
        "_err": err,
        "_p_aligned": p_aligned,
        "_p_gt": p_gt,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("estimate", help="estimated trajectory (TUM, IMU poses)")
    parser.add_argument("ground_truth", help="ground truth trajectory (TUM)")
    parser.add_argument("--lever-arm", type=float, nargs=3, default=None, metavar=("X", "Y", "Z"),
                        help="position of the tracked point (prism) in the IMU frame [m]")
    parser.add_argument("--max-gap", type=float, default=1.0,
                        help="do not interpolate across gaps larger than this [s]")
    parser.add_argument("--time-offset", default="0.0",
                        help="offset added to the estimate stamps [s], or 'auto' to estimate it")
    parser.add_argument("--plot", default=None, help="save a top-down/error plot to this file")
    parser.add_argument("--json", action="store_true", help="print the result as JSON")
    args = parser.parse_args()

    time_offset = args.time_offset if args.time_offset == "auto" else float(args.time_offset)
    res = evaluate(args.estimate, args.ground_truth, args.lever_arm, args.max_gap, time_offset)
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        axes[0].plot(res["_p_gt"][:, 0], res["_p_gt"][:, 1], "k-", lw=1.5, label="ground truth")
        axes[0].plot(res["_p_aligned"][:, 0], res["_p_aligned"][:, 1], "r-", lw=1.0,
                     label="estimate (aligned)")
        axes[0].set_aspect("equal", "datalim")
        axes[0].set_xlabel("x [m]")
        axes[0].set_ylabel("y [m]")
        axes[0].legend()
        axes[1].plot(res["_t"] - res["_t"][0], res["_err"], "r-", lw=1.0)
        axes[1].set_xlabel("time [s]")
        axes[1].set_ylabel("position error [m]")
        axes[1].set_title("ATE RMSE %.3f m" % res["rmse"])
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
    public = {k: v for k, v in res.items() if not k.startswith("_")}
    if args.json:
        json.dump(public, sys.stdout)
        print()
    else:
        print("ATE RMSE %.4f m | mean %.4f | median %.4f | max %.4f | pairs %d | "
              "GT length %.1f m (%.2f %%) | coverage %.1f %% | time offset %.3f s" %
              (public["rmse"], public["mean"], public["median"], public["max"], public["n_pairs"],
               public["gt_length"], public["rmse_percent"], 100 * public["coverage"],
               public["time_offset"]))


if __name__ == "__main__":
    main()
