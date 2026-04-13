import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

EPS = np.finfo(np.float32).eps * 4.0


def quat2mat(quaternion):
    q = np.asarray(quaternion, dtype=float)[[3, 0, 1, 2]].copy()
    n = np.dot(q, q)
    if n < EPS:
        return np.eye(3, dtype=float)

    q *= np.sqrt(2.0 / n)
    q2 = np.outer(q, q)
    return np.array(
        [
            [1.0 - q2[2, 2] - q2[3, 3], q2[1, 2] - q2[3, 0], q2[1, 3] + q2[2, 0]],
            [q2[1, 2] + q2[3, 0], 1.0 - q2[1, 1] - q2[3, 3], q2[2, 3] - q2[1, 0]],
            [q2[1, 3] - q2[2, 0], q2[2, 3] + q2[1, 0], 1.0 - q2[1, 1] - q2[2, 2]],
        ],
        dtype=float,
    )


def raw_msg_to_homo_mat(data):
    remote_pose = np.asarray(data[0], dtype=float)
    homo_mat = np.zeros((4, 4), dtype=float)
    homo_mat[:3, :3] = quat2mat(remote_pose[3:])
    homo_mat[:3, 3] = remote_pose[:3]
    homo_mat[3, 3] = 1.0
    return homo_mat


def set_equal_axes(ax, points):
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = (mins + maxs) * 0.5
    radius = max((maxs - mins).max() * 0.5, 1e-3)
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def slice_transforms(transforms, start_idx=0, end_idx=None, every_n=1):
    # start_idx = max(int(start_idx), 0)
    if end_idx is not None:
        end_idx = max(int(end_idx), start_idx)
    transforms = transforms[start_idx:end_idx]

    return transforms[:: max(int(every_n), 1)]


def plot_coordinate_frame_trajectory(ax, transforms, title, every_n=1, start_idx=0, end_idx=None):
    transforms = slice_transforms(
        np.asarray(transforms, dtype=float),
        start_idx=start_idx,
        end_idx=end_idx,
        every_n=every_n,
    )
    if len(transforms) == 0:
        ax.set_title(f"{title}\n(no samples)")
        return

    positions = transforms[:, :3, 3]
    rotations = transforms[:, :3, :3]
    axis_length = max(np.ptp(positions, axis=0).max() * 0.08, 0.02)

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

    ax.scatter(*positions[0], color="black", s=35)
    ax.scatter(*positions[-1], color="gold", edgecolors="black", s=45)
    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    set_equal_axes(ax, np.concatenate(frame_endpoints, axis=0))
    return samples


def load_franka_debug_transforms(path):
    logged = {
        "raw_msg_homo_mat": [],
        "homo_mat": [],
        "homo_mat_refl_y": [],
        "for_logging": [],
        "controller_origin_to_init": [],
        "robot_init_to_current": [],
        "robot_origin_to_init": [],
        "robot_origin_to_current": [],
    }

    with Path(path).open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            event = record.get("event")

            if event == "raw_msg" and "data" in record:
                logged["raw_msg_homo_mat"].append(raw_msg_to_homo_mat(record["data"]))
            elif event == "homo_mat" and "homo_mat" in record:
                logged["homo_mat"].append(record["homo_mat"])
            elif event == "homo_mat_refl_y" and "homo_mat" in record:
                logged["homo_mat_refl_y"].append(record["homo_mat"])
            elif event == "controller_tracking_transform":
                for_logging = record.get("for_logging")
                controller_origin_to_init = record.get("controller_origin_to_init")
                robot_init_to_current = record.get("robot_init_to_current")
                robot_origin_to_init = record.get("robot_origin_to_init")
                robot_origin_to_current = record.get("robot_origin_to_current")
                if for_logging is not None:
                    logged["for_logging"].append(
                        np.asarray(for_logging, dtype=float)
                    )
                if controller_origin_to_init is not None:
                    logged["controller_origin_to_init"].append(
                        np.asarray(controller_origin_to_init, dtype=float)
                    )
                if robot_init_to_current is not None:
                    logged["robot_init_to_current"].append(
                        np.asarray(robot_init_to_current, dtype=float)
                    )
                if robot_origin_to_init is not None:
                    logged["robot_origin_to_init"].append(
                        np.asarray(robot_origin_to_init, dtype=float)
                    )
                if robot_origin_to_current is not None:
                    logged["robot_origin_to_current"].append(
                        np.asarray(robot_origin_to_current, dtype=float)
                    )
                elif robot_origin_to_init is not None and robot_init_to_current is not None:
                    logged["robot_origin_to_current"].append(
                        np.asarray(robot_origin_to_init, dtype=float)
                        @ np.asarray(robot_init_to_current, dtype=float)
                    )

    return {key: np.asarray(value, dtype=float) for key, value in logged.items()}


def plot_logged_homo_mats(transform_sets, every_n=1, start_idx=0, end_idx=None):
    titles = [
        ("raw_msg_homo_mat", "Raw Msg Homo Mat"),
        ("homo_mat", "Remote Homo Mat"),
        ("homo_mat_refl_y", "Homo Mat Refl Y"),
        ("for_logging", "For Logging"),
        ("controller_origin_to_init", "Controller Origin To Init"),
        ("robot_init_to_current", "Robot Init To Current"),
        ("robot_origin_to_init", "Robot Origin To Init"),
        ("robot_origin_to_current", "Robot Origin To Current"),
    ]

    legend_handles = [
        Line2D([0], [0], color="0.35", linewidth=1.0, label="position path"),
        Line2D([0], [0], color="tab:red", linewidth=2.0, label="frame x"),
        Line2D([0], [0], color="tab:green", linewidth=2.0, label="frame y"),
        Line2D([0], [0], color="tab:blue", linewidth=2.0, label="frame z"),
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

    for key, title in titles:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(projection="3d")
        samples = plot_coordinate_frame_trajectory(
            ax,
            transform_sets[key],
            title=title,
            every_n=every_n,
            start_idx=start_idx,
            end_idx=end_idx,
        )
        fig.legend(handles=legend_handles, loc="upper center", ncol=6, bbox_to_anchor=(0.5, 0.98))
        if samples is not None:
            fig.colorbar(samples, ax=ax, pad=0.08, shrink=0.75, label="Sample")
        fig.tight_layout(rect=(0, 0, 1, 0.95))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--path",
        type=Path,
        default=Path.cwd() / "extracted_data" / "franka_debug.jsonl",
        help="Path to the JSONL debug log written by franka.py.",
    )
    parser.add_argument(
        "--every-n",
        type=int,
        default=1,
        help="Plot every Nth pose to reduce clutter.",
    )
    parser.add_argument(
        "--start-idx",
        type=int,
        default=0,
        help="Start plotting at this timestep index.",
    )
    parser.add_argument(
        "--end-idx",
        type=int,
        default=None,
        help="Stop plotting at this timestep index (exclusive).",
    )
    args = parser.parse_args()

    plot_logged_homo_mats(
        load_franka_debug_transforms(args.path),
        every_n=args.every_n,
        start_idx=args.start_idx,
        end_idx=args.end_idx,
    )
    plt.show()
