from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np


@dataclass(frozen=True)
class ControllerReport:
    controller_name: str
    accepted: bool
    message: str
    program_path: str | None = None
    sample_count: int = 0


class RobotController(Protocol):
    """Future real-hardware controller boundary."""

    controller_name: str

    def send_program(self, program: dict[str, Any]) -> ControllerReport:
        """Send a robot program to a controller.

        Real robots should implement this method with vendor SDK, TCP, ROS,
        Modbus, or serial communication. The app currently uses DryRunController.
        """


class DryRunController:
    controller_name = "DryRunController"

    def send_program(self, program: dict[str, Any]) -> ControllerReport:
        points = program.get("trajectory_points", [])
        return ControllerReport(
            controller_name=self.controller_name,
            accepted=True,
            message=f"虚拟控制器已接收程序，但未连接真实机械臂。轨迹点数: {len(points)}。",
            sample_count=len(points),
        )


class RobotProgramService:
    """Export simulation output into a hardware-ready program format."""

    def build_program(self, results: dict[str, Any]) -> dict[str, Any]:
        track = results["cartesian_track"]
        config = results.get("active_config", {})
        robot_name = results["fk_demo"]["robot_name"]
        time_s = np.asarray(track["time_s"], dtype=float)
        q_deg = np.asarray(track["q_deg"], dtype=float)
        positions = np.asarray(track["position_m"], dtype=float)
        desired_positions = np.asarray(track["desired_position_m"], dtype=float)

        trajectory_points = []
        for idx, time_value in enumerate(time_s):
            trajectory_points.append(
                {
                    "index": int(idx),
                    "time_s": float(time_value),
                    "joint_deg": np.round(q_deg[idx], 6).tolist(),
                    "tcp_position_m": np.round(positions[idx], 6).tolist(),
                    "desired_tcp_position_m": np.round(desired_positions[idx], 6).tolist(),
                }
            )

        return {
            "format": "robot_task_program_v1",
            "controller_mode": "dry_run_only",
            "robot_name": robot_name,
            "joint_order": [link["joint"] for link in config.get("links", [])],
            "joint_unit": "degree",
            "position_unit": "meter",
            "source": {
                "trajectory_type": track["summary"]["trajectory_type"],
                "task_blocks": track.get("task_blocks", []),
                "commands": track.get("commands", []),
            },
            "diagnostics": {
                "max_position_error_m": float(track["metrics"]["max_position_error_m"]),
                "mean_position_error_m": float(track["metrics"]["mean_position_error_m"]),
                "ik_failure_count": int(track["metrics"].get("ik_failure_count", 0)),
                "near_singular_count": int(track["metrics"].get("near_singular_count", 0)),
                "max_joint_step_deg": float(track["metrics"].get("max_joint_step_deg", 0.0)),
            },
            "safety_note": "This file is generated from simulation. Validate speed, limits, tooling, workspace, and emergency stop before connecting real hardware.",
            "trajectory_points": trajectory_points,
        }

    def export_program(self, results: dict[str, Any], export_root: str | Path) -> Path:
        export_dir = Path(export_root)
        export_dir.mkdir(parents=True, exist_ok=True)
        program = self.build_program(results)
        output_path = export_dir / "robot_program_dry_run.json"
        output_path.write_text(json.dumps(program, ensure_ascii=False, indent=2), encoding="utf-8")
        return output_path

    def dry_run_send(self, results: dict[str, Any]) -> ControllerReport:
        program = self.build_program(results)
        return DryRunController().send_program(program)
