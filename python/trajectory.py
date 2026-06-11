from __future__ import annotations

from typing import Any

import numpy as np

from .robot_model import SixDofRobot, rpy_deg_from_transform


def time_vector(duration_s: float, steps: int) -> np.ndarray:
    return np.linspace(0.0, float(duration_s), int(steps))


def quintic_blend(tau: np.ndarray) -> np.ndarray:
    return 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5


def joint_ptp_trajectory(q_start_rad: np.ndarray, q_goal_rad: np.ndarray, duration_s: float, steps: int) -> tuple[np.ndarray, np.ndarray]:
    t = time_vector(duration_s, steps)
    tau = np.divide(t - t[0], max(t[-1] - t[0], np.finfo(float).eps))
    s = quintic_blend(tau)[:, None]
    q_traj = np.asarray(q_start_rad, dtype=float) + (np.asarray(q_goal_rad, dtype=float) - np.asarray(q_start_rad, dtype=float)) * s
    return t, q_traj


def plane_unit_vectors(plane: str, center_hint_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    plane = plane.lower()
    hint = np.zeros(3, dtype=float)

    if plane == "yz":
        hint[1:] = center_hint_m[1:]
        if np.linalg.norm(hint[1:]) < 1e-9:
            hint = np.array([0.0, 1.0, 0.0], dtype=float)
        radial = hint / np.linalg.norm(hint)
        tangent = np.array([0.0, -radial[2], radial[1]], dtype=float)
    elif plane == "xz":
        hint[[0, 2]] = center_hint_m[[0, 2]]
        if np.linalg.norm(hint[[0, 2]]) < 1e-9:
            hint = np.array([1.0, 0.0, 0.0], dtype=float)
        radial = hint / np.linalg.norm(hint)
        tangent = np.array([-radial[2], 0.0, radial[0]], dtype=float)
    else:
        hint[:2] = center_hint_m[:2]
        if np.linalg.norm(hint[:2]) < 1e-9:
            hint = np.array([1.0, 0.0, 0.0], dtype=float)
        radial = hint / np.linalg.norm(hint)
        tangent = np.array([-radial[1], radial[0], 0.0], dtype=float)

    tangent /= np.linalg.norm(tangent)
    return radial, tangent


def cartesian_circle_positions(
    start_position_m: np.ndarray,
    radius_m: float,
    center_hint_m: np.ndarray,
    plane: str,
    steps: int,
) -> np.ndarray:
    radial, tangent = plane_unit_vectors(plane, np.asarray(center_hint_m, dtype=float))
    center = np.asarray(start_position_m, dtype=float) - float(radius_m) * radial
    phases = np.linspace(0.0, 2.0 * np.pi, int(steps))
    positions = np.array(
        [center + radius_m * (radial * np.cos(phi) + tangent * np.sin(phi)) for phi in phases],
        dtype=float,
    )
    positions[0] = np.asarray(start_position_m, dtype=float)
    return positions


def poses_from_positions(positions_m: np.ndarray, reference_transform: np.ndarray) -> list[np.ndarray]:
    rotation = reference_transform[:3, :3]
    poses = []
    for position in positions_m:
        target = np.eye(4)
        target[:3, :3] = rotation
        target[:3, 3] = position
        poses.append(target)
    return poses


def reachable_reference_trajectory(
    robot: SixDofRobot,
    q_start_rad: np.ndarray,
    duration_s: float,
    steps: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t = time_vector(duration_s, steps)
    phases = np.linspace(0.0, 2.0 * np.pi, int(steps))
    amplitudes = np.deg2rad(np.array([8.0, 10.0, 8.0, 12.0, 6.0, 15.0], dtype=float))
    phase_offsets = np.array([0.0, np.pi / 6.0, np.pi / 3.0, 0.0, np.pi / 4.0, np.pi / 2.0], dtype=float)

    q_traj = np.zeros((int(steps), robot.n), dtype=float)
    for idx, phase in enumerate(phases):
        q_candidate = np.asarray(q_start_rad, dtype=float) + amplitudes * np.sin(phase + phase_offsets)
        q_traj[idx] = robot.clamp_to_limits(q_candidate)

    positions = np.array([robot.fkine(q)[:3, 3] for q in q_traj], dtype=float)
    return t, q_traj, positions


def cartesian_track_trajectory(
    robot: SixDofRobot,
    q_start_rad: np.ndarray,
    duration_s: float,
    steps: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    defaults = robot.config["simulation_defaults"]
    start_transform = robot.fkine(q_start_rad)
    positions = cartesian_circle_positions(
        start_transform[:3, 3],
        float(defaults["cartesian_track_radius_m"]),
        np.asarray(defaults["cartesian_track_center_offset_m"], dtype=float),
        str(defaults["cartesian_track_plane"]),
        int(steps),
    )

    q_traj = np.zeros((int(steps), robot.n), dtype=float)
    q_traj[0] = np.asarray(q_start_rad, dtype=float)
    try:
        for idx in range(1, int(steps)):
            q_traj[idx] = robot.inverse_kinematics_position_only(positions[idx], q0_rad=q_traj[idx - 1])
    except RuntimeError:
        return reachable_reference_trajectory(robot, q_start_rad, duration_s, steps)

    return time_vector(duration_s, steps), q_traj, positions


def evaluate_trajectory(
    robot: SixDofRobot,
    time_s: np.ndarray,
    q_traj_rad: np.ndarray,
    desired_positions_m: np.ndarray | None,
    source: str,
    trajectory_type: str,
) -> dict[str, Any]:
    transforms = []
    positions = []
    rpy_values = []

    for q in q_traj_rad:
        transform = robot.fkine(q)
        transforms.append(transform)
        positions.append(transform[:3, 3])
        rpy_values.append(rpy_deg_from_transform(transform))

    positions_arr = np.asarray(positions, dtype=float)
    rpy_arr = np.asarray(rpy_values, dtype=float)
    transforms_arr = np.asarray(transforms, dtype=float)

    if desired_positions_m is None:
        errors = np.zeros(len(time_s), dtype=float)
        desired_positions_m = positions_arr.copy()
    else:
        desired_positions_m = np.asarray(desired_positions_m, dtype=float)
        errors = np.linalg.norm(positions_arr - desired_positions_m, axis=1)

    return {
        "summary": {
            "robot_name": robot.robot_name,
            "source": source,
            "trajectory_type": trajectory_type,
            "time_span_s": float(time_s[-1] - time_s[0]),
            "samples": int(len(time_s)),
        },
        "time_s": np.asarray(time_s, dtype=float),
        "q_rad": np.asarray(q_traj_rad, dtype=float),
        "q_deg": np.rad2deg(q_traj_rad),
        "position_m": positions_arr,
        "desired_position_m": desired_positions_m,
        "rpy_deg": rpy_arr,
        "transforms": transforms_arr,
        "metrics": {
            "max_position_error_m": float(np.max(errors)),
            "mean_position_error_m": float(np.mean(errors)),
        },
    }
