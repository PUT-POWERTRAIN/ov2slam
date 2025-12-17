#!/usr/bin/env python3

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("TkAgg")  # must be set before importing pyplot
import matplotlib.pyplot as plt
import numpy as np


@dataclass
class Trajectory:
    stamp: np.ndarray  # float seconds
    pos: np.ndarray  # Nx3


def read_pose_csv(path: Path) -> Trajectory:
    stamps = []
    positions = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            sec = int(row["stamp_sec"])
            nsec = int(row["stamp_nanosec"])
            stamps.append(sec + nsec * 1e-9)
            positions.append(
                (
                    float(row["pos_x"]),
                    float(row["pos_y"]),
                    float(row["pos_z"]),
                )
            )
    if not positions:
        return Trajectory(np.array([]), np.zeros((0, 3)))
    return Trajectory(np.array(stamps, dtype=np.float64), np.array(positions, dtype=np.float64))


def set_axes_equal(ax):
    # Make 3D plot axes have equal scale.
    limits = np.array(
        [
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d(),
        ],
        dtype=np.float64,
    )
    spans = limits[:, 1] - limits[:, 0]
    centers = limits.mean(axis=1)
    radius = 0.5 * np.max(spans)
    ax.set_xlim3d([centers[0] - radius, centers[0] + radius])
    ax.set_ylim3d([centers[1] - radius, centers[1] + radius])
    ax.set_zlim3d([centers[2] - radius, centers[2] + radius])


def main():
    dir_path = Path("trajectory")
    ap = argparse.ArgumentParser(description="Plot 3D trajectories from trajectory_csv_logger outputs.")
    ap.add_argument("--vo", type=Path, default=dir_path / Path("vo_pose.csv"), help="Path to VO CSV (PoseStamped)")
    ap.add_argument("--gt", type=Path, default=dir_path / Path("gt_pose.csv"), help="Path to GT CSV (PoseStamped)")
    ap.add_argument("--title", type=str, default="Trajectories (3D)", help="Plot title")
    ap.add_argument("--no-gt", action="store_true", help="Disable plotting GT")
    ap.add_argument("--no-vo", action="store_true", help="Disable plotting VO")
    args = ap.parse_args()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    if not args.no_vo and args.vo.exists():
        vo = read_pose_csv(args.vo)
        if vo.pos.shape[0] > 0:
            ax.plot(vo.pos[:, 0], vo.pos[:, 1], vo.pos[:, 2], label=f"VO ({vo.pos.shape[0]} pts)", linewidth=1.0)
        else:
            print(f"[warn] VO file has no rows: {args.vo}")
    elif not args.no_vo:
        print(f"[warn] VO file not found: {args.vo}")

    if not args.no_gt and args.gt.exists():
        gt = read_pose_csv(args.gt)
        if gt.pos.shape[0] > 0:
            ax.plot(gt.pos[:, 0], gt.pos[:, 1], gt.pos[:, 2], label=f"GT ({gt.pos.shape[0]} pts)", linewidth=1.0)
        else:
            print(f"[warn] GT file has no rows: {args.gt}")
    elif not args.no_gt:
        print(f"[warn] GT file not found: {args.gt}")

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title(args.title)
    ax.legend()
    set_axes_equal(ax)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

