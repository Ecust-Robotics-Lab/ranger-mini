#!/usr/bin/env python3
"""Create geometric review statistics and plots for a Lightning experiment."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from check_slam_map import pcd_xyz


def yaw_deg(row, offset):
    qx, qy, qz, qw = row[offset + 3:offset + 7]
    return math.degrees(math.atan2(2 * (qw * qz + qx * qy),
                                   1 - 2 * (qy * qy + qz * qz)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment", type=Path)
    args = parser.parse_args()
    exp = args.experiment.resolve()
    map_dir = exp / "data/new_map"
    cloud = pcd_xyz(map_dir / "global.pcd")
    trajectory = np.loadtxt(map_dir / "keyframes.txt", ndmin=2)
    lio = trajectory[:, 2:5]
    opt = trajectory[:, 9:12]
    start, end = opt[0], opt[-1]
    steps = np.linalg.norm(np.diff(opt, axis=0), axis=1)
    yaw_start, yaw_end = yaw_deg(trajectory[0], 9), yaw_deg(trajectory[-1], 9)
    yaw_delta = (yaw_end - yaw_start + 180) % 360 - 180
    z_percentiles = np.percentile(cloud[:, 2], [0, 1, 5, 25, 50, 75, 95, 99, 100])
    result = {
        "experiment": str(exp),
        "map_points": int(len(cloud)),
        "keyframes": int(len(trajectory)),
        "optimized_path_length_m": float(steps.sum()),
        "optimized_start_xyz_m": start.tolist(),
        "optimized_end_xyz_m": end.tolist(),
        "start_end_translation_m": float(np.linalg.norm(end - start)),
        "start_end_planar_m": float(np.linalg.norm((end - start)[:2])),
        "start_end_height_m": float(end[2] - start[2]),
        "start_yaw_deg": yaw_start,
        "end_yaw_deg": yaw_end,
        "start_end_yaw_delta_deg": yaw_delta,
        "optimized_trajectory_z_min_m": float(opt[:, 2].min()),
        "optimized_trajectory_z_max_m": float(opt[:, 2].max()),
        "optimized_trajectory_z_span_m": float(np.ptp(opt[:, 2])),
        "max_lio_to_optimized_translation_m": float(np.linalg.norm(lio - opt, axis=1).max()),
        "cloud_z_percentiles_m": dict(zip(["p0", "p1", "p5", "p25", "p50", "p75", "p95", "p99", "p100"], map(float, z_percentiles))),
        "review_limits": [
            "No surveyed reference distance was supplied, so absolute scale cannot be accepted.",
            "Point-cloud plots can reveal gross duplication but do not replace on-site scene comparison.",
            "A returned position with different yaw is not a full pose closure."
        ]
    }
    (exp / "geometric_review.json").write_text(json.dumps(result, indent=2) + "\n")

    fig, ax = plt.subplots(figsize=(10, 9), constrained_layout=True)
    clipped = np.clip(cloud[:, 2], np.percentile(cloud[:, 2], 2), np.percentile(cloud[:, 2], 98))
    points = ax.scatter(cloud[:, 0], cloud[:, 1], c=clipped, s=0.8, cmap="turbo", alpha=0.65)
    ax.plot(opt[:, 0], opt[:, 1], "k.-", linewidth=1.2, markersize=4, label="optimized trajectory")
    ax.plot(start[0], start[1], "go", markersize=9, label="start")
    ax.plot(end[0], end[1], "rx", markersize=9, label="end")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_title("Lightning 3D map — XY review (color: z)")
    ax.legend()
    fig.colorbar(points, ax=ax, label="z (m), clipped 2–98 percentile")
    fig.savefig(exp / "map_review_xy.png", dpi=180)
    plt.close(fig)

    fig = plt.figure(figsize=(11, 8), constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")
    stride = max(1, len(cloud) // 15000)
    sample = cloud[::stride]
    ax.scatter(sample[:, 0], sample[:, 1], sample[:, 2], c=sample[:, 2], s=0.7, cmap="turbo", alpha=0.6)
    ax.plot(opt[:, 0], opt[:, 1], opt[:, 2], "k.-", linewidth=1.4, markersize=4)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title("Lightning 3D map and optimized trajectory")
    fig.savefig(exp / "map_review_3d.png", dpi=180)
    plt.close(fig)

    bands = [(-1.0, 0.3), (0.3, 2.0), (2.0, 2.7)]
    fig, axes = plt.subplots(1, 3, figsize=(18, 6), constrained_layout=True)
    for ax, (low, high) in zip(axes, bands):
        selected = cloud[(cloud[:, 2] >= low) & (cloud[:, 2] < high)]
        ax.scatter(selected[:, 0], selected[:, 1], s=1.0, color="navy", alpha=0.55)
        ax.plot(opt[:, 0], opt[:, 1], "r.-", linewidth=1.1, markersize=3)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(-6, 6)
        ax.set_ylim(-5, 6)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title(f"{low:.1f} ≤ z < {high:.1f} m ({len(selected)} points)")
        ax.grid(alpha=0.2)
    fig.suptitle("Central map slices for wall/ground/ceiling review")
    fig.savefig(exp / "map_review_slices.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 5), constrained_layout=True)
    ax.hist(cloud[:, 2], bins=140, range=(-1.0, 4.0), color="steelblue")
    ax.set_xlabel("map z (m)")
    ax.set_ylabel("point count")
    ax.set_title("Global map height distribution")
    fig.savefig(exp / "map_review_height_histogram.png", dpi=180)
    plt.close(fig)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
