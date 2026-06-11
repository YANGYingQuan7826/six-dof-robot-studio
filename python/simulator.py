from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import numpy as np

from .robot_model import SixDofRobot, load_robot_config
from .trajectory import cartesian_track_trajectory, evaluate_trajectory, joint_ptp_trajectory


def _to_serializable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: _to_serializable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_serializable(item) for item in value]
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    return value


def export_results(output_dir: str | Path, result: dict[str, Any]) -> None:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    time_s = np.asarray(result["time_s"], dtype=float)
    q_deg = np.asarray(result["q_deg"], dtype=float)
    q_rad = np.asarray(result["q_rad"], dtype=float)
    position_m = np.asarray(result["position_m"], dtype=float)
    rpy_deg = np.asarray(result["rpy_deg"], dtype=float)

    with (output_path / "joint_trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        header = ["time_s"] + [f"q{i + 1}_deg" for i in range(q_deg.shape[1])] + [f"q{i + 1}_rad" for i in range(q_rad.shape[1])]
        writer.writerow(header)
        for idx, time_value in enumerate(time_s):
            writer.writerow([time_value, *q_deg[idx].tolist(), *q_rad[idx].tolist()])

    with (output_path / "pose_trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg"])
        for idx, time_value in enumerate(time_s):
            writer.writerow([time_value, *position_m[idx].tolist(), *rpy_deg[idx].tolist()])

    (output_path / "simulation_result.json").write_text(
        json.dumps(_to_serializable(result), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def run_fk_demo(config_source: str | Path | dict[str, Any] | None = None) -> dict[str, Any]:
    config = load_robot_config(config_source)
    robot = SixDofRobot(config)
    q_init_rad = np.deg2rad(np.asarray(config["default_state"]["q_init_deg"], dtype=float))
    q_goal_rad = np.deg2rad(np.asarray(config["default_state"]["q_goal_deg"], dtype=float))

    return {
        "robot_name": robot.robot_name,
        "initial_pose": robot.pose_dict(q_init_rad),
        "goal_pose": robot.pose_dict(q_goal_rad),
        "links": config["links"],
    }


def run_default_simulation(
    config_source: str | Path | dict[str, Any] | None = None,
    output_root: str | Path | None = None,
) -> dict[str, Any]:
    config = load_robot_config(config_source)
    robot = SixDofRobot(config)

    q_init_rad = np.deg2rad(np.asarray(config["default_state"]["q_init_deg"], dtype=float))
    q_goal_rad = np.deg2rad(np.asarray(config["default_state"]["q_goal_deg"], dtype=float))
    duration_s = float(config["simulation_defaults"]["duration_s"])
    steps = int(config["simulation_defaults"]["steps"])

    time_ptp, q_ptp = joint_ptp_trajectory(q_init_rad, q_goal_rad, duration_s, steps)
    ptp_result = evaluate_trajectory(robot, time_ptp, q_ptp, None, source="python", trajectory_type="ptp")

    time_track, q_track, desired_positions = cartesian_track_trajectory(robot, q_init_rad, duration_s, steps)
    track_result = evaluate_trajectory(
        robot,
        time_track,
        q_track,
        desired_positions,
        source="python",
        trajectory_type="cartesian_track",
    )

    outputs = {
        "fk_demo": run_fk_demo(config),
        "ptp": ptp_result,
        "cartesian_track": track_result,
    }

    if output_root is not None:
        output_root = Path(output_root)
        export_results(output_root / "python_ptp", ptp_result)
        export_results(output_root / "python_cartesian_track", track_result)
        (output_root / "python_fk_demo.json").write_text(
            json.dumps(_to_serializable(outputs["fk_demo"]), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    return outputs
