from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from python.robot_model import SixDofRobot, rpy_deg_from_transform, transform_from_rpy_translation


@dataclass(frozen=True)
class IKRequest:
    target_position_m: list[float]
    target_rpy_deg: list[float] | None = None
    q_seed_deg: list[float] | None = None
    mode: str = "position_only"


@dataclass(frozen=True)
class IKResult:
    success: bool
    q_solution_deg: list[float]
    position_error_m: float
    orientation_error_deg: float | None
    message: str


class KinematicsService:
    """Application-facing API for FK/IK operations.

    ViewModels call this service instead of reaching into the numeric robot model
    directly. That keeps UI state handling separate from kinematics algorithms.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        self.robot = SixDofRobot(config)
        self.config = config

    def solve_inverse(self, request: IKRequest) -> IKResult:
        q_seed_rad = self._seed_rad(request.q_seed_deg)
        target_position = np.asarray(request.target_position_m, dtype=float).reshape(3)

        try:
            if request.mode == "full_pose":
                target_transform = self._target_transform(target_position, request.target_rpy_deg)
                q_solution = self.robot.inverse_kinematics(target_transform, q0_rad=q_seed_rad)
            else:
                q_solution = self.robot.inverse_kinematics_position_only(target_position, q0_rad=q_seed_rad)

            current_transform = self.robot.fkine(q_solution)
            position_error = float(np.linalg.norm(current_transform[:3, 3] - target_position))
            orientation_error = self._orientation_error_deg(current_transform, request.target_rpy_deg)
            return IKResult(
                success=True,
                q_solution_deg=np.rad2deg(q_solution).round(6).tolist(),
                position_error_m=position_error,
                orientation_error_deg=orientation_error,
                message="IK converged.",
            )
        except Exception as exc:  # noqa: BLE001
            return IKResult(
                success=False,
                q_solution_deg=np.rad2deg(q_seed_rad).round(6).tolist(),
                position_error_m=float("inf"),
                orientation_error_deg=None,
                message=str(exc),
            )

    def _seed_rad(self, q_seed_deg: list[float] | None) -> np.ndarray:
        if q_seed_deg is None:
            q_seed_deg = self.config["default_state"]["q_init_deg"]
        return np.deg2rad(np.asarray(q_seed_deg, dtype=float))

    def _target_transform(self, target_position: np.ndarray, target_rpy_deg: list[float] | None) -> np.ndarray:
        if target_rpy_deg is None:
            seed = self._seed_rad(None)
            target_transform = self.robot.fkine(seed)
            target_transform[:3, 3] = target_position
            return target_transform

        target_transform = transform_from_rpy_translation(target_rpy_deg, target_position.tolist())
        return target_transform

    def _orientation_error_deg(self, current_transform: np.ndarray, target_rpy_deg: list[float] | None) -> float | None:
        if target_rpy_deg is None:
            return None
        current_rpy = rpy_deg_from_transform(current_transform)
        return float(np.linalg.norm(current_rpy - np.asarray(target_rpy_deg, dtype=float)))
