"""Python helpers around the GeoFIK Franka IK bindings."""

from __future__ import annotations

import numpy as np

from openteach.ik import _geofik

T_8_E = np.array(
    [
        [0.70710678, 0.70710678, 0.0, 0.0],
        [-0.70710678, 0.70710678, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.1034],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

def solve_geofik_q7(current_q, target_pose, q7=None, ee_frame="E"):
    """Return the GeoFIK solution closest to the current Franka joints."""
    current_q = np.asarray(current_q, dtype=np.float64).reshape(7)
    if q7 is None:
        q7 = current_q[6]

    nsols, qsols = solve_q7_from_pose(target_pose, q7=q7, ee_frame=ee_frame)
    solution = nearest_solution(qsols, current_q)
    return solution

def solve_geofik_q4(current_q, target_pose, q4=None, ee_frame="E"):
    """Return the q4-parameterized GeoFIK solution closest to the current joints."""
    current_q = np.asarray(current_q, dtype=np.float64).reshape(7)
    if q4 is None:
        q4 = current_q[3]

    nsols, qsols = solve_q4_from_pose(target_pose, q4=q4, ee_frame=ee_frame)
    solution = nearest_solution(qsols, current_q)
    return solution

def solve_geofik_q6(current_q, target_pose, q6=None, ee_frame="E"):
    """Return the q6-parameterized GeoFIK solution closest to the current joints."""
    current_q = np.asarray(current_q, dtype=np.float64).reshape(7)
    if q6 is None:
        q6 = current_q[5]

    nsols, qsols = solve_q6_from_pose(target_pose, q6=q6, ee_frame=ee_frame)
    solution = nearest_solution(qsols, current_q)
    return solution

def solve_geofik_swivel(current_q, target_pose, theta=None, ee_frame="E"):
    """Return the swivel-parameterized GeoFIK solution closest to the current joints."""
    current_q = np.asarray(current_q, dtype=np.float64).reshape(7)
    if theta is None:
        theta = franka_swivel(current_q)

    nsols, qsols = solve_swivel_from_pose(target_pose, theta=theta, ee_frame=ee_frame)
    solution = nearest_solution_for_joint_with_lower_penalty(
        qsols,
        current_q,
        joint_idx=0,
        penalty_joint_idx=1,
        lower_limit=-1.5,
        penalty=10.0,
    )
    return solution


def pose_to_position_rotation(pose: np.ndarray, ee_frame: str = "E") -> tuple[np.ndarray, np.ndarray]:
    """Return GeoFIK position and rotation inputs from a 4x4 pose matrix.

    GeoFIK always solves IK for Franka frame E. If the input pose is for
    panda_link8, pass ``ee_frame="8"`` and the static 8-to-E transform is
    applied before solving.
    """
    pose = np.asarray(pose, dtype=np.float64)
    if pose.shape != (4, 4):
        raise ValueError("pose must have shape (4, 4)")

    if ee_frame == "8":
        pose = pose @ T_8_E
    elif ee_frame != "E":
        raise ValueError('ee_frame must be "E" or "8"')

    return pose[:3, 3].copy(), pose[:3, :3].copy()


def solve_q7_from_pose(
    pose: np.ndarray,
    q7: float,
    *,
    ee_frame: str = "E",
    q1_sing: float = np.pi / 2,
) -> tuple[int, np.ndarray]:
    position, rotation = pose_to_position_rotation(pose, ee_frame=ee_frame)
    return _geofik.franka_ik_q7(position, rotation, q7, q1_sing=q1_sing)


def solve_q6_from_pose(
    pose: np.ndarray,
    q6: float,
    *,
    ee_frame: str = "E",
    q1_sing: float = np.pi / 2,
    q7_sing: float = 0.0,
) -> tuple[int, np.ndarray]:
    position, rotation = pose_to_position_rotation(pose, ee_frame=ee_frame)
    return _geofik.franka_ik_q6(position, rotation, q6, q1_sing=q1_sing, q7_sing=q7_sing)


def solve_q4_from_pose(
    pose: np.ndarray,
    q4: float,
    *,
    ee_frame: str = "E",
    q1_sing: float = np.pi / 2,
    q7_sing: float = 0.0,
) -> tuple[int, np.ndarray]:
    position, rotation = pose_to_position_rotation(pose, ee_frame=ee_frame)
    return _geofik.franka_ik_q4(position, rotation, q4, q1_sing=q1_sing, q7_sing=q7_sing)


def solve_swivel_from_pose(
    pose: np.ndarray,
    theta: float,
    *,
    ee_frame: str = "E",
    q1_sing: float = np.pi / 2,
    n_points: int = 500,
    n_fine_search: int = 3,
) -> tuple[int, np.ndarray]:
    position, rotation = pose_to_position_rotation(pose, ee_frame=ee_frame)
    return _geofik.franka_ik_swivel(
        position,
        rotation,
        theta,
        q1_sing=q1_sing,
        n_points=n_points,
        n_fine_search=n_fine_search,
    )


def finite_solutions(qsols: np.ndarray) -> np.ndarray:
    qsols = np.asarray(qsols, dtype=np.float64)
    return qsols[np.isfinite(qsols).all(axis=1)]


def nearest_solution(qsols: np.ndarray, reference_q: np.ndarray) -> np.ndarray | None:
    valid_qsols = finite_solutions(qsols)
    if len(valid_qsols) == 0:
        return None
    reference_q = np.asarray(reference_q, dtype=np.float64).reshape(7)
    return valid_qsols[np.argmin(np.linalg.norm(valid_qsols - reference_q, axis=1))]


def nearest_solution_for_joint(
    qsols: np.ndarray,
    reference_q: np.ndarray,
    joint_idx: int = 0,
) -> np.ndarray | None:
    valid_qsols = finite_solutions(qsols)
    if len(valid_qsols) == 0:
        return None

    reference_q = np.asarray(reference_q, dtype=np.float64).reshape(7)
    if not 0 <= joint_idx < reference_q.shape[0]:
        raise ValueError("joint_idx must be in [0, 6]")

    joint_distances = np.abs(valid_qsols[:, joint_idx] - reference_q[joint_idx])
    total_distances = np.linalg.norm(valid_qsols - reference_q, axis=1)
    return valid_qsols[np.lexsort((total_distances, joint_distances))[0]]


def nearest_solution_for_joint_with_lower_penalty(
    qsols: np.ndarray,
    reference_q: np.ndarray,
    joint_idx: int,
    penalty_joint_idx: int,
    lower_limit: float,
    penalty: float = 1.0,
) -> np.ndarray | None:
    valid_qsols = finite_solutions(qsols)
    if len(valid_qsols) == 0:
        return None

    reference_q = np.asarray(reference_q, dtype=np.float64).reshape(7)
    if not 0 <= joint_idx < reference_q.shape[0]:
        raise ValueError("joint_idx must be in [0, 6]")
    if not 0 <= penalty_joint_idx < reference_q.shape[0]:
        raise ValueError("penalty_joint_idx must be in [0, 6]")

    lower_penalties = np.maximum(lower_limit - valid_qsols[:, penalty_joint_idx], 0.0) * penalty
    joint_distances = np.abs(valid_qsols[:, joint_idx] - reference_q[joint_idx]) + lower_penalties
    total_distances = np.linalg.norm(valid_qsols - reference_q, axis=1) + lower_penalties
    return valid_qsols[np.lexsort((total_distances, joint_distances))[0]]


franka_ik_q7 = _geofik.franka_ik_q7
franka_ik_q6 = _geofik.franka_ik_q6
franka_ik_q4 = _geofik.franka_ik_q4
franka_ik_swivel = _geofik.franka_ik_swivel
franka_J_ik_q7 = _geofik.franka_J_ik_q7
franka_J_ik_q6 = _geofik.franka_J_ik_q6
franka_J_ik_q4 = _geofik.franka_J_ik_q4
franka_J_ik_swivel = _geofik.franka_J_ik_swivel
J_from_q = _geofik.J_from_q
franka_fk = _geofik.franka_fk
franka_swivel = _geofik.franka_swivel
