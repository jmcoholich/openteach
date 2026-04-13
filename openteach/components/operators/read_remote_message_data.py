import argparse
from ast import literal_eval
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

EPS = np.finfo(np.float32).eps * 4.0


def quat2mat_batch(quaternions):
    """Match deoxys.utils.quat2mat for batches of (x, y, z, w) quaternions."""
    q = np.asarray(quaternions, dtype=float)[:, [3, 0, 1, 2]].copy()
    rotations = np.repeat(np.eye(3, dtype=float)[None, :, :], len(q), axis=0)

    norms = np.einsum("ij,ij->i", q, q)
    valid = norms >= EPS
    if not np.any(valid):
        return rotations

    q = q[valid] * np.sqrt(2.0 / norms[valid])[:, None]
    q2 = q[:, :, None] * q[:, None, :]

    rotations[valid, 0, 0] = 1.0 - q2[:, 2, 2] - q2[:, 3, 3]
    rotations[valid, 0, 1] = q2[:, 1, 2] - q2[:, 3, 0]
    rotations[valid, 0, 2] = q2[:, 1, 3] + q2[:, 2, 0]
    rotations[valid, 1, 0] = q2[:, 1, 2] + q2[:, 3, 0]
    rotations[valid, 1, 1] = 1.0 - q2[:, 1, 1] - q2[:, 3, 3]
    rotations[valid, 1, 2] = q2[:, 2, 3] - q2[:, 1, 0]
    rotations[valid, 2, 0] = q2[:, 1, 3] - q2[:, 2, 0]
    rotations[valid, 2, 1] = q2[:, 2, 3] + q2[:, 1, 0]
    rotations[valid, 2, 2] = 1.0 - q2[:, 1, 1] - q2[:, 2, 2]
    return rotations


def load_remote_message_data(path=None):
    if path is None:
        path = Path(__file__).with_name("data.txt")
    rows = [literal_eval(line) for line in Path(path).read_text().splitlines() if line.strip()]
    poses = np.asarray([row[0] for row in rows], dtype=float)

    transforms = np.zeros((len(poses), 4, 4), dtype=float)
    transforms[:, :3, :3] = quat2mat_batch(poses[:, 3:])
    transforms[:, :3, 3] = poses[:, :3]
    transforms[:, 3, 3] = 1.0
    x_rot_90 = np.array([[1, 0, 0, 0],[0, 0, -1, 0],[0, 1, 0, 0], [0,0,0,1]])
    transforms = x_rot_90 @ transforms
    return transforms


def set_equal_axes(ax, points):
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = (mins + maxs) * 0.5
    radius = max((maxs - mins).max() * 0.5, 1e-3)
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def slice_transforms(transforms, max_timesteps=None, every_n=1):
    if max_timesteps is not None:
        max_timesteps = max(int(max_timesteps), 0)
        transforms = transforms[:max_timesteps]

    every_n = max(int(every_n), 1)
    return transforms[::every_n]


def plot_coordinate_frame_trajectory(transforms, every_n=1, max_timesteps=None):
    transforms = slice_transforms(
        transforms,
        max_timesteps=max_timesteps,
        every_n=every_n,
    )
    if len(transforms) == 0:
        raise ValueError("No transforms available to plot after applying filters.")

    positions = transforms[:, :3, 3]
    rotations = transforms[:, :3, :3]

    axis_length = max(np.ptp(positions, axis=0).max() * 0.08, 0.02)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(projection="3d")
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], color="0.35", linewidth=1.0)
    samples = ax.scatter(
        positions[:, 0],
        positions[:, 1],
        positions[:, 2],
        c=np.arange(len(positions)),
        cmap="viridis",
        s=18,
    )

    axis_colors = ("tab:red", "tab:green", "tab:blue")
    axis_labels = ("frame x", "frame y", "frame z")
    frame_endpoints = [positions]
    for axis_idx, color in enumerate(axis_colors):
        directions = rotations[:, :, axis_idx] * axis_length
        ax.quiver(
            positions[:, 0],
            positions[:, 1],
            positions[:, 2],
            directions[:, 0],
            directions[:, 1],
            directions[:, 2],
            color=color,
            linewidth=0.8,
            arrow_length_ratio=0.2,
            normalize=False,
        )
        frame_endpoints.append(positions + directions)

    ax.scatter(*positions[0], color="black", s=40, label="start")
    ax.scatter(*positions[-1], color="gold", edgecolors="black", s=50, label="end")
    ax.set_title("Remote Message Pose Trajectory")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    set_equal_axes(ax, np.concatenate(frame_endpoints, axis=0))

    legend_handles = [
        Line2D([0], [0], color="0.35", linewidth=1.0, label="position path"),
        Line2D([0], [0], color=axis_colors[0], linewidth=2.0, label=axis_labels[0]),
        Line2D([0], [0], color=axis_colors[1], linewidth=2.0, label=axis_labels[1]),
        Line2D([0], [0], color=axis_colors[2], linewidth=2.0, label=axis_labels[2]),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="black", markersize=7, label="start"),
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor="gold",
            markeredgecolor="black",
            markersize=7,
            label="end",
        ),
    ]
    ax.legend(handles=legend_handles, loc="best")
    fig.colorbar(samples, ax=ax, pad=0.08, label="Sample")
    fig.tight_layout()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--path",
        type=Path,
        default=Path(__file__).with_name("data_new.txt"),
        help="Path to the recorded remote message data file.",
    )
    parser.add_argument(
        "--every-n",
        type=int,
        default=1,
        help="Plot every Nth pose to reduce clutter.",
    )
    parser.add_argument(
        "--max-timesteps",
        type=int,
        default=None,
        help="Only plot the first M timesteps.",
    )
    args = parser.parse_args()

    plot_coordinate_frame_trajectory(
        load_remote_message_data(args.path),
        every_n=args.every_n,
        max_timesteps=args.max_timesteps,
    )
    plt.show()
