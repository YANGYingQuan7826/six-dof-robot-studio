from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from python.robot_model import SixDofRobot
from python.trajectory import evaluate_trajectory, time_vector


@dataclass(frozen=True)
class TrajectoryCommand:
    trajectory_type: str
    axis: str = "z"
    radius_m: float = 0.04
    length_m: float = 0.08
    rise_m: float = 0.08
    turns: float = 1.0
    # 水平圆（axis=z，圆在 xy 平面）：锁定末端世界 z；None 表示沿用起点高度
    circle_plane_z_m: float | None = None


class TrajectoryService:
    """Generate user-requested Cartesian trajectories and solve their IK track."""

    def __init__(self) -> None:
        self._workspace_map_cache: dict[str, np.ndarray] = {}

    def run_command(self, config: dict[str, Any], command: TrajectoryCommand) -> dict[str, Any]:
        return self.run_commands(config, [command])

    def run_commands(self, config: dict[str, Any], commands: list[TrajectoryCommand]) -> dict[str, Any]:
        if not commands:
            raise ValueError("At least one trajectory command is required.")

        robot = SixDofRobot(config)
        q_default = np.deg2rad(np.asarray(config["default_state"]["q_init_deg"], dtype=float))
        duration_s = float(config["simulation_defaults"]["duration_s"])
        steps_per_command = min(int(config["simulation_defaults"]["steps"]), 51)
        q_start, auto_start = self._select_auto_start_pose(robot, q_default, commands, steps_per_command)
        precheck = self.precheck_commands(robot, q_start, commands, steps_per_command)
        precheck["auto_start"] = auto_start
        self._apply_ik_preview_to_precheck(precheck)

        q_segments = []
        desired_segments = []
        used_commands = []
        requested_commands = []
        segment_diagnostics = []
        failure_count = 0
        q_current = q_start
        start_position = robot.fkine(q_current)[:3, 3]

        for command_idx, command in enumerate(commands):
            desired_positions, q_track, segment_failures, used_command = self._best_track(
                robot, q_current, start_position, command, steps_per_command
            )
            if command_idx > 0:
                desired_positions = desired_positions[1:]
                q_track = q_track[1:]
            desired_segments.append(desired_positions)
            q_segments.append(q_track)
            failure_count += segment_failures
            used_commands.append(used_command.__dict__)
            requested_commands.append(command.__dict__)
            segment_diagnostics.append(
                self._segment_diagnostics(robot, command_idx, desired_positions, q_track, segment_failures, command, used_command)
            )
            q_current = q_track[-1]
            start_position = robot.fkine(q_current)[:3, 3]

        desired_positions = np.vstack(desired_segments)
        q_track = np.vstack(q_segments)
        total_steps = len(q_track)
        total_duration = duration_s * max(len(commands), 1)
        result = evaluate_trajectory(
            robot,
            time_vector(total_duration, total_steps),
            q_track,
            desired_positions,
            source="python",
            trajectory_type="task_sequence",
        )
        result["metrics"]["ik_failure_count"] = int(failure_count)
        result["metrics"]["ik_failure_ratio"] = float(failure_count / max(total_steps - 1, 1))
        condition_numbers = self._condition_numbers(robot, q_track)
        result["metrics"]["max_jacobian_condition"] = float(np.max(condition_numbers))
        result["metrics"]["near_singular_count"] = int(np.sum(condition_numbers > 200.0))
        result["metrics"].update(self._joint_motion_metrics(q_track))
        result["metrics"].update(self._environment_motion_metrics(robot, q_track))
        result["command"] = used_commands[-1]
        result["requested_command"] = requested_commands[-1]
        result["commands"] = used_commands
        result["requested_commands"] = requested_commands
        result["segment_diagnostics"] = segment_diagnostics
        result["precheck"] = precheck
        return result

    def precheck_config_commands(self, config: dict[str, Any], commands: list[TrajectoryCommand]) -> dict[str, Any]:
        if not commands:
            raise ValueError("At least one trajectory command is required.")
        robot = SixDofRobot(config)
        q_default = np.deg2rad(np.asarray(config["default_state"]["q_init_deg"], dtype=float))
        steps = min(int(config["simulation_defaults"]["steps"]), 51)
        q_start, auto_start = self._select_auto_start_pose(
            robot,
            q_default,
            commands,
            steps,
            use_tracking_preview=False,
        )
        precheck = self.precheck_commands(robot, q_start, commands, steps)
        precheck["auto_start"] = auto_start
        self._apply_ik_preview_to_precheck(precheck)
        return precheck

    def _apply_ik_preview_to_precheck(self, precheck: dict[str, Any]) -> None:
        preview = (precheck.get("auto_start") or {}).get("ik_preview") or {}
        failure_ratio = float(preview.get("failure_ratio", 1.0))
        max_error = float(preview.get("max_error_m", float("inf")))
        max_step = float(preview.get("max_joint_step_deg", float("inf")))
        if failure_ratio > 0.0 or max_error > 0.010:
            return

        precheck["ik_preview_passed"] = True
        precheck["risk_level"] = "low" if max_step <= 25.0 else "medium"
        for segment in precheck.get("segments", []):
            segment["recommended_scale"] = 1.0

    def _select_auto_start_pose(
        self,
        robot: SixDofRobot,
        q_default: np.ndarray,
        commands: list[TrajectoryCommand],
        steps: int,
        *,
        use_tracking_preview: bool = True,
        preview_candidate_limit: int = 4,
        preview_steps: int = 11,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        candidates = self._auto_start_candidates(robot, q_default)
        preliminary: list[tuple[float, np.ndarray, dict[str, Any]]] = []

        for q_candidate in candidates:
            q_candidate = robot.clamp_to_limits(q_candidate)
            precheck = self.precheck_commands(robot, q_candidate, commands, steps)
            risk_score = {"low": 0.0, "medium": 1.0, "high": 2.0}.get(precheck["risk_level"], 2.0)
            condition = float(np.linalg.cond(robot.numerical_jacobian(q_candidate)[:3, :]))
            q_motion = float(np.linalg.norm(q_candidate - q_default))
            g_margin = float(precheck.get("ground_margin_m_at_start", 0.0))
            score = (
                10.0 * risk_score
                + 18.0 * max(0.0, -g_margin)
                + 3.0 * float(precheck.get("outside_workspace_map_ratio", 0.0))
                + 2.0 * float(precheck.get("max_workspace_map_distance_m", 0.0))
                + 0.2 * max(0.0, -float(precheck.get("workspace_margin_m", 0.0)))
                + 0.002 * min(condition, 500.0)
                + 0.03 * q_motion
            )
            preliminary.append((score, q_candidate, precheck))

        if not preliminary:
            best_q = robot.clamp_to_limits(q_default)
            chosen_position = robot.fkine(best_q)[:3, 3]
            default_position = robot.fkine(q_default)[:3, 3]
            return best_q, {
                "enabled": True,
                "q_init_deg": np.round(np.rad2deg(best_q), 3).tolist(),
                "default_q_init_deg": np.round(np.rad2deg(q_default), 3).tolist(),
                "start_position_m": np.round(chosen_position, 6).tolist(),
                "default_start_position_m": np.round(default_position, 6).tolist(),
                "risk_level": "unknown",
                "score": 0.0,
                "candidate_count": int(len(candidates)),
                "ik_preview": {},
                "reason": "未生成候选姿态，沿用默认初始姿态。",
            }

        if not use_tracking_preview:
            best_score, best_q, best_precheck = min(preliminary, key=lambda item: item[0])
            chosen_position = robot.fkine(best_q)[:3, 3]
            default_position = robot.fkine(q_default)[:3, 3]
            return best_q, {
                "enabled": True,
                "q_init_deg": np.round(np.rad2deg(best_q), 3).tolist(),
                "default_q_init_deg": np.round(np.rad2deg(q_default), 3).tolist(),
                "start_position_m": np.round(chosen_position, 6).tolist(),
                "default_start_position_m": np.round(default_position, 6).tolist(),
                "risk_level": best_precheck["risk_level"],
                "score": float(best_score),
                "candidate_count": int(len(candidates)),
                "ik_preview": {},
                "reason": "快速预检查仅按工作空间、地面约束和奇异性风险选择起点；正式执行时会再做 IK 试跑。",
            }

        # The workspace map is intentionally conservative and can rank two
        # large horizontal circles incorrectly. Re-score the best-looking
        # candidates by actually tracking a short IK preview at the requested
        # scale before deciding the auto start pose.
        best_score = float("inf")
        best_q = robot.clamp_to_limits(q_default)
        best_precheck: dict[str, Any] | None = None
        best_execution: dict[str, Any] | None = None
        for pre_score, q_candidate, precheck in sorted(preliminary, key=lambda item: item[0])[:preview_candidate_limit]:
            execution = self._score_start_pose_tracking(robot, q_candidate, commands, min(steps, preview_steps))
            score = (
                pre_score
                + 45.0 * float(execution["failure_ratio"])
                + 80.0 * max(0.0, float(execution["max_error_m"]) - 0.010)
                + 0.03 * float(execution["max_joint_step_deg"])
            )
            if score < best_score:
                best_score = score
                best_q = q_candidate
                best_precheck = precheck
                best_execution = execution

        chosen_position = robot.fkine(best_q)[:3, 3]
        default_position = robot.fkine(q_default)[:3, 3]
        return best_q, {
            "enabled": True,
            "q_init_deg": np.round(np.rad2deg(best_q), 3).tolist(),
            "default_q_init_deg": np.round(np.rad2deg(q_default), 3).tolist(),
            "start_position_m": np.round(chosen_position, 6).tolist(),
            "default_start_position_m": np.round(default_position, 6).tolist(),
            "risk_level": best_precheck["risk_level"] if best_precheck else "unknown",
            "score": float(best_score),
            "candidate_count": int(len(candidates)),
            "ik_preview": best_execution or {},
            "reason": "从候选姿态中选择预检查风险较低且短轨迹 IK 试跑误差较小的起点。",
        }

    def _score_start_pose_tracking(
        self,
        robot: SixDofRobot,
        q_start: np.ndarray,
        commands: list[TrajectoryCommand],
        steps: int,
    ) -> dict[str, float]:
        q_current = np.asarray(q_start, dtype=float).reshape(robot.n)
        start_position = robot.fkine(q_current)[:3, 3]
        total_segments = 0
        total_failures = 0
        max_error = 0.0
        max_joint_step = 0.0

        for command in commands:
            desired_positions = self._desired_positions(
                start_position,
                command,
                steps,
                base_position=robot.base[:3, 3],
            )
            q_track, failure_count = self._track_positions(robot, q_current, desired_positions, command=command)
            actual_positions = np.asarray([robot.fkine(q)[:3, 3] for q in q_track], dtype=float)
            errors = np.linalg.norm(actual_positions - desired_positions, axis=1)
            max_error = max(max_error, float(np.max(errors)))
            if len(q_track) > 1:
                joint_steps = np.linalg.norm(np.diff(q_track, axis=0), axis=1)
                max_joint_step = max(max_joint_step, float(np.max(np.rad2deg(joint_steps))))
            total_failures += int(failure_count)
            total_segments += max(len(desired_positions) - 1, 1)
            q_current = q_track[-1]
            start_position = robot.fkine(q_current)[:3, 3]

        return {
            "failure_ratio": float(total_failures / max(total_segments, 1)),
            "max_error_m": float(max_error),
            "max_joint_step_deg": float(max_joint_step),
        }

    def _auto_start_candidates(self, robot: SixDofRobot, q_default: np.ndarray) -> list[np.ndarray]:
        candidate_deg = [
            np.rad2deg(q_default).tolist(),
            [41.37, -35.43, 47.2, 43.55, 49.29, 7.18],
            [0.0, -45.0, 90.0, 0.0, 45.0, 0.0],
            [0.0, -55.0, 95.0, 0.0, 45.0, 0.0],
            [20.0, -50.0, 90.0, 0.0, 45.0, 0.0],
            [-20.0, -50.0, 90.0, 0.0, 45.0, 0.0],
            [35.0, -40.0, 65.0, 25.0, 45.0, 0.0],
            [-35.0, -40.0, 65.0, -25.0, 45.0, 0.0],
        ]

        # Rotate the base around several directions while keeping a folded,
        # non-singular arm shape. This gives circle/line tasks more room.
        for base_yaw in [-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0]:
            candidate_deg.append([base_yaw, -45.0, 80.0, 0.0, 45.0, 0.0])

        out = [np.deg2rad(np.asarray(q_deg, dtype=float)) for q_deg in candidate_deg]
        rng = np.random.default_rng(20260509)
        ql = robot.qlim_rad
        added = 0
        for _ in range(1200):
            if added >= 10:
                break
            q_rand = robot.clamp_to_limits(
                np.asarray(
                    [
                        rng.uniform(ql[i, 0], ql[i, 1]) for i in range(robot.n)
                    ],
                    dtype=float,
                )
            )
            if robot.min_checked_joint_clearance_above_floor_m(q_rand) > 0.0:
                out.append(q_rand.copy())
                added += 1
        if added == 0:
            best_q = None
            best_c = -1e9
            for _ in range(1500):
                q_rand = robot.clamp_to_limits(
                    np.asarray(
                        [
                            rng.uniform(ql[i, 0], ql[i, 1]) for i in range(robot.n)
                        ],
                        dtype=float,
                    )
                )
                c = robot.min_checked_joint_clearance_above_floor_m(q_rand)
                if c > best_c:
                    best_c, best_q = c, q_rand.copy()
            if best_q is not None:
                out.append(best_q)
        return out

    def precheck_commands(
        self,
        robot: SixDofRobot,
        q_start: np.ndarray,
        commands: list[TrajectoryCommand],
        steps: int,
    ) -> dict[str, Any]:
        start_position = robot.fkine(q_start)[:3, 3]
        base_position = robot.base[:3, 3]
        workspace_radius = self._estimated_workspace_radius(robot)
        workspace_points = self._workspace_map(robot)
        requested_segments = []
        all_positions = []
        current_position = start_position

        for idx, command in enumerate(commands, start=1):
            positions = self._desired_positions(current_position, command, steps, base_position=base_position)
            distances = np.linalg.norm(positions - base_position, axis=1)
            map_distances = self._nearest_workspace_distances(positions, workspace_points)
            map_scale = self._workspace_map_scale(command, map_distances)
            radius_scale = self._recommended_scale(command, workspace_radius, float(np.max(distances)))
            requested_segments.append(
                {
                    "index": idx,
                    "trajectory_type": command.trajectory_type,
                    "axis": command.axis,
                    "requested_extent_m": self._command_extent(command),
                    "max_distance_from_base_m": float(np.max(distances)),
                    "workspace_margin_m": float(workspace_radius - np.max(distances)),
                    "max_workspace_map_distance_m": float(np.max(map_distances)),
                    "mean_workspace_map_distance_m": float(np.mean(map_distances)),
                    "outside_workspace_map_ratio": float(np.mean(map_distances > 0.06)),
                    "recommended_scale": float(min(radius_scale, map_scale)),
                }
            )
            all_positions.append(positions)
            current_position = positions[-1]

        all_positions_arr = np.vstack(all_positions) if all_positions else start_position.reshape(1, 3)
        distances = np.linalg.norm(all_positions_arr - base_position, axis=1)
        map_distances = self._nearest_workspace_distances(all_positions_arr, workspace_points)
        max_distance = float(np.max(distances))
        margin = float(workspace_radius - max_distance)
        trajectory_center = np.mean(all_positions_arr, axis=0)
        posture = self._recommend_initial_posture(robot, trajectory_center)
        center_shift = self._recommend_center_shift(base_position, trajectory_center, workspace_radius, max_distance)
        z_floor = robot.ground_plane_z_m()
        z_constraint = robot.joint_z_constraint_floor_m()
        min_j_h = robot.min_checked_joint_height_m(q_start)
        ground_margin_m = float(min_j_h - z_constraint)
        min_clear_start = robot.min_non_adjacent_segment_separation_raw_m(q_start)
        warn_clear_m = float(robot._environment.get("min_clearance_warn_mm", 30.0)) * 1e-3
        risk_level = self._precheck_risk_level(
            requested_segments,
            margin,
            float(np.max(map_distances)),
            ground_margin_m=ground_margin_m,
            min_clearance_m=min_clear_start,
            warn_clearance_m=warn_clear_m,
        )

        return {
            "risk_level": risk_level,
            "estimated_workspace_radius_m": float(workspace_radius),
            "max_requested_distance_from_base_m": max_distance,
            "workspace_margin_m": margin,
            "workspace_map_samples": int(len(workspace_points)),
            "max_workspace_map_distance_m": float(np.max(map_distances)),
            "mean_workspace_map_distance_m": float(np.mean(map_distances)),
            "outside_workspace_map_ratio": float(np.mean(map_distances > 0.06)),
            "trajectory_center_m": np.round(trajectory_center, 6).tolist(),
            "recommended_center_shift_m": np.round(center_shift, 6).tolist(),
            "recommended_q_init_deg": posture["q_init_deg"],
            "recommended_q_init_reason": posture["reason"],
            "segments": requested_segments,
            "ground_plane_z_m": float(z_floor),
            "joint_z_constraint_floor_m": float(z_constraint),
            "min_checked_joint_height_m_at_start": float(min_j_h),
            "ground_margin_m_at_start": float(ground_margin_m),
            "min_link_segment_separation_raw_m_at_start": float(min_clear_start)
            if np.isfinite(min_clear_start)
            else None,
            "clearance_warn_m": float(warn_clear_m),
        }

    def _estimated_workspace_radius(self, robot: SixDofRobot) -> float:
        tool_offset = float(np.linalg.norm(robot.tool[:3, 3]))
        return 0.82 * float(np.sum(np.abs(robot.a_m)) + np.sum(np.abs(robot.d_m)) + tool_offset)

    def _workspace_map(self, robot: SixDofRobot) -> np.ndarray:
        key = self._workspace_map_key(robot)
        cached = self._workspace_map_cache.get(key)
        if cached is not None:
            return cached

        rng = np.random.default_rng(20260508)
        qlim = robot.qlim_rad
        samples = [np.deg2rad(np.asarray(robot.config["default_state"]["q_init_deg"], dtype=float))]

        # Bias samples toward common non-singular industrial arm postures.
        for _ in range(1500):
            q = np.array(
                [
                    rng.uniform(qlim[0, 0], qlim[0, 1]),
                    rng.uniform(np.deg2rad(-80.0), np.deg2rad(25.0)),
                    rng.uniform(np.deg2rad(20.0), np.deg2rad(120.0)),
                    rng.uniform(qlim[3, 0], qlim[3, 1]),
                    rng.uniform(np.deg2rad(-90.0), np.deg2rad(90.0)),
                    rng.uniform(np.deg2rad(-180.0), np.deg2rad(180.0)),
                ],
                dtype=float,
            )
            samples.append(robot.clamp_to_limits(q))

        points = np.asarray([robot.fkine(q)[:3, 3] for q in samples], dtype=float)
        self._workspace_map_cache[key] = points
        return points

    def _workspace_map_key(self, robot: SixDofRobot) -> str:
        values = np.concatenate([robot.a_m, robot.d_m, robot.alpha_rad, robot.qlim_rad.reshape(-1)])
        return "|".join(f"{value:.6f}" for value in values)

    def _nearest_workspace_distances(self, positions: np.ndarray, workspace_points: np.ndarray) -> np.ndarray:
        positions = np.asarray(positions, dtype=float)
        nearest = []
        chunk_size = 64
        for start in range(0, len(positions), chunk_size):
            chunk = positions[start : start + chunk_size]
            distances = np.linalg.norm(chunk[:, None, :] - workspace_points[None, :, :], axis=2)
            nearest.extend(np.min(distances, axis=1).tolist())
        return np.asarray(nearest, dtype=float)

    def _workspace_map_scale(self, command: TrajectoryCommand, map_distances: np.ndarray) -> float:
        max_distance = float(np.max(map_distances))
        outside_ratio = float(np.mean(map_distances > 0.06))
        if outside_ratio > 0.35 or max_distance > 0.12:
            return 0.15 if command.trajectory_type.lower() == "circle" else 0.25
        if outside_ratio > 0.15 or max_distance > 0.09:
            return 0.5
        if outside_ratio > 0.05 or max_distance > 0.07:
            return 0.75
        return 1.0

    def _command_extent(self, command: TrajectoryCommand) -> float:
        trajectory_type = command.trajectory_type.lower()
        if trajectory_type == "line":
            return float(command.length_m)
        if trajectory_type == "circle":
            return 2.0 * float(command.radius_m)
        if trajectory_type == "spatial_curve":
            return float(np.hypot(2.0 * command.radius_m, command.rise_m))
        return 0.0

    def _recommended_scale(self, command: TrajectoryCommand, workspace_radius: float, max_distance: float) -> float:
        extent = max(self._command_extent(command), 1e-9)
        margin_scale = 1.0 if max_distance <= workspace_radius else max(0.25, 1.0 - (max_distance - workspace_radius) / extent)
        stable_extent = 0.045 if command.trajectory_type.lower() == "line" else 0.40
        stability_scale = min(1.0, stable_extent / extent)
        min_scale = 0.15 if command.trajectory_type.lower() == "circle" else 0.25
        return float(max(min_scale, min(margin_scale, stability_scale)))

    def _recommend_center_shift(
        self,
        base_position: np.ndarray,
        trajectory_center: np.ndarray,
        workspace_radius: float,
        max_distance: float,
    ) -> np.ndarray:
        if max_distance <= 0.78 * workspace_radius:
            return np.zeros(3, dtype=float)
        inward = base_position - trajectory_center
        inward_norm = float(np.linalg.norm(inward))
        if inward_norm < 1e-9:
            return np.zeros(3, dtype=float)
        shift_distance = min(0.08, max(0.0, max_distance - 0.72 * workspace_radius))
        return shift_distance * inward / inward_norm

    def _recommend_initial_posture(self, robot: SixDofRobot, trajectory_center: np.ndarray) -> dict[str, Any]:
        candidate_deg = [
            robot.config["default_state"]["q_init_deg"],
            [0.0, -45.0, 75.0, 0.0, 35.0, 0.0],
            [20.0, -50.0, 90.0, 0.0, 45.0, 0.0],
            [-20.0, -40.0, 80.0, 0.0, 40.0, 0.0],
            [0.0, -30.0, 60.0, 0.0, 30.0, 0.0],
        ]
        best_score = float("inf")
        best_q = np.asarray(candidate_deg[0], dtype=float)
        for q_deg in candidate_deg:
            q = robot.clamp_to_limits(np.deg2rad(np.asarray(q_deg, dtype=float)))
            position = robot.fkine(q)[:3, 3]
            condition = float(np.linalg.cond(robot.numerical_jacobian(q)[:3, :]))
            score = float(np.linalg.norm(position - trajectory_center)) + 0.001 * min(condition, 500.0)
            if score < best_score:
                best_score = score
                best_q = np.rad2deg(q)
        return {
            "q_init_deg": np.round(best_q, 3).tolist(),
            "reason": "选择末端更靠近轨迹中心且位置雅可比条件数较低的候选姿态。",
        }

    def _precheck_risk_level(
        self,
        segments: list[dict[str, Any]],
        margin: float,
        max_map_distance: float,
        ground_margin_m: float | None = None,
        min_clearance_m: float | None = None,
        warn_clearance_m: float = 0.03,
    ) -> str:
        def rank(level: str) -> int:
            return {"low": 0, "medium": 1, "high": 2}.get(level, 2)

        base = "low"
        if margin < 0.0 or max_map_distance > 0.12 or any(segment["recommended_scale"] <= 0.35 for segment in segments):
            base = "high"
        elif margin < 0.08 or max_map_distance > 0.08 or any(segment["recommended_scale"] < 0.75 for segment in segments):
            base = "medium"

        extra = "low"
        if ground_margin_m is not None and ground_margin_m <= 0.0:
            extra = "high"
        elif ground_margin_m is not None and ground_margin_m < 0.002:
            extra = "medium"

        if min_clearance_m is not None and np.isfinite(min_clearance_m):
            if min_clearance_m < max(0.005, 0.5 * warn_clearance_m):
                extra = "high"
            elif extra != "high" and min_clearance_m < warn_clearance_m:
                extra = "medium"

        order = max(rank(base), rank(extra))
        return "high" if order == 2 else "medium" if order == 1 else "low"

    def _environment_motion_metrics(self, robot: SixDofRobot, q_track: np.ndarray) -> dict[str, float]:
        if len(q_track) == 0:
            return {
                "min_checked_joint_height_m": float("nan"),
                "min_checked_joint_clearance_above_floor_m": float("nan"),
                "min_link_clearance_m": float("nan"),
                "min_link_segment_separation_raw_m": float("nan"),
                "ground_plane_z_m": robot.ground_plane_z_m(),
                "joint_z_constraint_floor_m": robot.joint_z_constraint_floor_m(),
            }
        heights = np.asarray([robot.min_checked_joint_height_m(q) for q in q_track], dtype=float)
        clr_floor = np.asarray([robot.min_checked_joint_clearance_above_floor_m(q) for q in q_track], dtype=float)
        clears = np.asarray([robot.min_non_adjacent_segment_clearance_m(q) for q in q_track], dtype=float)
        raw_clears = np.asarray([robot.min_non_adjacent_segment_separation_raw_m(q) for q in q_track], dtype=float)
        clears = np.where(np.isfinite(clears), clears, np.nan)
        raw_clears = np.where(np.isfinite(raw_clears), raw_clears, np.nan)
        return {
            "min_checked_joint_height_m": float(np.nanmin(heights)),
            "min_checked_joint_clearance_above_floor_m": float(np.nanmin(clr_floor)),
            "min_link_clearance_m": float(np.nanmin(clears)) if np.any(np.isfinite(clears)) else float("nan"),
            "min_link_segment_separation_raw_m": float(np.nanmin(raw_clears))
            if np.any(np.isfinite(raw_clears))
            else float("nan"),
            "ground_plane_z_m": float(robot.ground_plane_z_m()),
            "joint_z_constraint_floor_m": float(robot.joint_z_constraint_floor_m()),
        }

    def _segment_diagnostics(
        self,
        robot: SixDofRobot,
        command_idx: int,
        desired_positions: np.ndarray,
        q_track: np.ndarray,
        failure_count: int,
        requested_command: TrajectoryCommand,
        used_command: TrajectoryCommand,
    ) -> dict[str, Any]:
        actual_positions = np.asarray([robot.fkine(q)[:3, 3] for q in q_track], dtype=float)
        errors = np.linalg.norm(actual_positions - desired_positions, axis=1)
        return {
            "index": int(command_idx + 1),
            "trajectory_type": used_command.trajectory_type,
            "axis": used_command.axis,
            "samples": int(len(q_track)),
            "ik_failure_count": int(failure_count),
            "ik_failure_ratio": float(failure_count / max(len(q_track) - 1, 1)),
            "max_position_error_m": float(np.max(errors)),
            "mean_position_error_m": float(np.mean(errors)),
            "requested_command": requested_command.__dict__,
            "used_command": used_command.__dict__,
            "auto_scaled": requested_command != used_command,
        }

    def _joint_motion_metrics(self, q_track: np.ndarray) -> dict[str, float]:
        if len(q_track) < 2:
            return {
                "max_joint_step_deg": 0.0,
                "mean_joint_step_deg": 0.0,
                "rms_joint_step_deg": 0.0,
            }
        joint_steps = np.linalg.norm(np.diff(q_track, axis=0), axis=1)
        joint_steps_deg = np.rad2deg(joint_steps)
        return {
            "max_joint_step_deg": float(np.max(joint_steps_deg)),
            "mean_joint_step_deg": float(np.mean(joint_steps_deg)),
            "rms_joint_step_deg": float(np.sqrt(np.mean(joint_steps_deg**2))),
        }

    def _condition_numbers(self, robot: SixDofRobot, q_track: np.ndarray) -> np.ndarray:
        values = []
        for q in q_track:
            jacobian = robot.numerical_jacobian(q)[:3, :]
            try:
                value = float(np.linalg.cond(jacobian))
            except np.linalg.LinAlgError:
                value = float("inf")
            if not np.isfinite(value):
                value = 1e9
            values.append(value)
        return np.asarray(values, dtype=float)

    def _best_track(
        self,
        robot: SixDofRobot,
        q_start: np.ndarray,
        start_position: np.ndarray,
        command: TrajectoryCommand,
        steps: int,
    ) -> tuple[np.ndarray, np.ndarray, int, TrajectoryCommand]:
        best_score = float("inf")
        best_data: tuple[np.ndarray, np.ndarray, int, TrajectoryCommand] | None = None
        acceptable_error = 0.010 if command.trajectory_type.lower() == "circle" else 0.006

        for scale in self._candidate_scales(command):
            candidate = TrajectoryCommand(
                trajectory_type=command.trajectory_type,
                axis=command.axis,
                radius_m=command.radius_m * scale,
                length_m=command.length_m * scale,
                rise_m=command.rise_m * scale,
                turns=command.turns,
                circle_plane_z_m=command.circle_plane_z_m,
            )
            desired_positions = self._desired_positions(start_position, candidate, steps, base_position=robot.base[:3, 3])
            q_track, failure_count = self._track_positions(robot, q_start, desired_positions, command=candidate)
            actual_positions = np.asarray([robot.fkine(q)[:3, 3] for q in q_track], dtype=float)
            errors = np.linalg.norm(actual_positions - desired_positions, axis=1)
            max_error = float(np.max(errors))
            if failure_count == 0 and max_error <= acceptable_error:
                return desired_positions, q_track, failure_count, candidate

            score = max_error + 0.02 * float(failure_count) - 0.001 * float(scale)
            if score < best_score:
                best_score = score
                best_data = (desired_positions, q_track, failure_count, candidate)

        if best_data is None:
            raise RuntimeError("No trajectory candidate was generated.")
        return best_data

    def _candidate_scales(self, command: TrajectoryCommand) -> list[float]:
        if command.trajectory_type.lower() == "line":
            return [1.0, 0.7, 0.5, 0.3]
        if command.trajectory_type.lower() == "circle":
            return [1.0, 0.75, 0.5, 0.35, 0.25, 0.15]
        return [1.0, 0.7, 0.5, 0.3]

    def _desired_positions(
        self,
        start_position: np.ndarray,
        command: TrajectoryCommand,
        steps: int,
        base_position: np.ndarray | None = None,
    ) -> np.ndarray:
        trajectory_type = command.trajectory_type.lower()
        if trajectory_type == "line":
            return self._line_positions(start_position, command.axis, command.length_m, steps)
        if trajectory_type == "circle":
            return self._circle_positions(
                start_position,
                command.axis,
                command.radius_m,
                steps,
                base_position=base_position,
                circle_plane_z_m=command.circle_plane_z_m,
            )
        if trajectory_type == "spatial_curve":
            return self._spatial_curve_positions(start_position, command.axis, command.radius_m, command.rise_m, command.turns, steps)
        raise ValueError(f"Unsupported trajectory type: {command.trajectory_type}")

    def _line_positions(self, start_position: np.ndarray, axis: str, length_m: float, steps: int) -> np.ndarray:
        direction = self._axis_vector(axis)
        s_values = np.linspace(0.0, 1.0, steps)
        return np.asarray([start_position + float(length_m) * s * direction for s in s_values], dtype=float)

    def _circle_positions(
        self,
        start_position: np.ndarray,
        axis: str,
        radius_m: float,
        steps: int,
        base_position: np.ndarray | None = None,
        circle_plane_z_m: float | None = None,
    ) -> np.ndarray:
        normal = self._axis_vector(axis)
        basis_u, basis_v = self._orthogonal_basis(normal)
        radius_m = float(radius_m)
        axis_id = axis.lower().strip()

        start_for_geom = np.asarray(start_position, dtype=float).reshape(3).copy()
        if circle_plane_z_m is not None and axis_id == "z":
            start_for_geom[2] = float(circle_plane_z_m)

        radial = self._circle_start_radial(start_for_geom, base_position, normal, basis_u, basis_v)
        tangent = np.cross(normal, radial)
        tangent = tangent / np.linalg.norm(tangent)
        center = start_for_geom - radius_m * radial
        phases = np.linspace(0.0, 2.0 * np.pi, steps)
        positions = np.asarray([center + radius_m * (radial * np.cos(phi) + tangent * np.sin(phi)) for phi in phases], dtype=float)
        positions[0] = start_for_geom
        return positions

    def _circle_start_radial(
        self,
        start_position: np.ndarray,
        base_position: np.ndarray | None,
        normal: np.ndarray,
        basis_u: np.ndarray,
        basis_v: np.ndarray,
    ) -> np.ndarray:
        if base_position is None:
            return basis_u

        start_position = np.asarray(start_position, dtype=float)
        base_position = np.asarray(base_position, dtype=float)
        normal = np.asarray(normal, dtype=float)
        normal = normal / np.linalg.norm(normal)

        # Make the circle center move inward, so the far side of a large circle
        # is less likely to be pushed outside the reachable workspace.
        outward = start_position - base_position
        radial = outward - np.dot(outward, normal) * normal
        if float(np.linalg.norm(radial)) < 1e-9:
            radial = basis_u
        radial = radial / np.linalg.norm(radial)
        return radial

    def _spatial_curve_positions(
        self,
        start_position: np.ndarray,
        axis: str,
        radius_m: float,
        rise_m: float,
        turns: float,
        steps: int,
    ) -> np.ndarray:
        axis_unit = self._axis_vector(axis)
        basis_u, basis_v = self._orthogonal_basis(axis_unit)
        radius_m = float(radius_m)
        origin = start_position - radius_m * basis_u
        s_values = np.linspace(0.0, 1.0, steps)
        positions = []
        for s in s_values:
            phase = 2.0 * np.pi * float(turns) * s
            positions.append(origin + radius_m * (basis_u * np.cos(phase) + basis_v * np.sin(phase)) + float(rise_m) * s * axis_unit)
        positions = np.asarray(positions, dtype=float)
        positions[0] = start_position
        return positions

    def _track_positions(
        self,
        robot: SixDofRobot,
        q_start: np.ndarray,
        desired_positions: np.ndarray,
        command: TrajectoryCommand | None = None,
    ) -> tuple[np.ndarray, int]:
        q_track = np.zeros((len(desired_positions), robot.n), dtype=float)
        q_track[0] = q_start
        failure_count = 0
        is_circle = command is not None and command.trajectory_type.lower() == "circle"
        max_step_deg = 9.0 if is_circle else 3.0
        smoothness_gain = 0.035 if is_circle else 0.08
        residual_limit = 0.014 if is_circle else 0.008
        for idx in range(1, len(desired_positions)):
            try:
                q_candidate = self._solve_position_step_dls(
                    robot,
                    q_track[idx - 1],
                    desired_positions[idx],
                    max_step_deg=max_step_deg,
                    smoothness_gain=smoothness_gain,
                )
                if bool(robot._environment.get("enforce_joint_above_ground", True)):
                    if robot.min_checked_joint_clearance_above_floor_m(q_candidate) <= 0.0:
                        raise RuntimeError("Joint height not strictly above configured ground plane.")
                residual = float(np.linalg.norm(robot.fkine(q_candidate)[:3, 3] - desired_positions[idx]))
                joint_step = float(np.linalg.norm(q_candidate - q_track[idx - 1]))
                if residual > residual_limit or joint_step > np.deg2rad(45.0):
                    raise RuntimeError("Rejected unstable IK step.")
                q_track[idx] = q_candidate
            except RuntimeError:
                failure_count += 1
                q_track[idx] = q_track[idx - 1]
        return q_track, failure_count

    def _solve_position_step_dls(
        self,
        robot: SixDofRobot,
        q_seed: np.ndarray,
        target_position: np.ndarray,
        max_step_deg: float = 3.0,
        smoothness_gain: float = 0.08,
    ) -> np.ndarray:
        q_reference = np.asarray(q_seed, dtype=float).reshape(robot.n)
        q_next = q_reference.copy()
        target_position = np.asarray(target_position, dtype=float).reshape(3)
        damping = 2e-3
        tolerance = 5e-4
        max_iterations = 70
        max_step_rad = np.deg2rad(max_step_deg)
        limit_gain = 0.025

        for _ in range(max_iterations):
            current_position = robot.fkine(q_next)[:3, 3]
            error = target_position - current_position
            if float(np.linalg.norm(error)) <= tolerance:
                break

            jacobian = robot.numerical_jacobian(q_next)[:3, :]
            condition_number = float(np.linalg.cond(jacobian))
            adaptive_damping = damping * (1.0 + min(condition_number / 120.0, 8.0))
            smoothness_target = q_reference - q_next
            limit_target = self._joint_limit_centering_target(robot, q_next)
            augmented_jacobian = np.vstack(
                [
                    jacobian,
                    smoothness_gain * np.eye(robot.n),
                    limit_gain * np.eye(robot.n),
                    adaptive_damping * np.eye(robot.n),
                ]
            )
            augmented_error = np.concatenate(
                [
                    error,
                    smoothness_gain * smoothness_target,
                    limit_gain * limit_target,
                    np.zeros(robot.n, dtype=float),
                ]
            )
            env = robot._environment
            pos_err = float(np.linalg.norm(error))
            gate = float(env.get("ik_ground_gate_pos_err_m", 0.005))
            if bool(env.get("soft_penalty_in_ik", True)) and pos_err < gate:
                gp, ggc = robot.ground_penalty_grad(q_next)
                if gp > 1e-9:
                    w_base = float(env.get("ik_ground_weight", 8.0))
                    beta = float(env.get("ik_ground_penalty_damp", 45.0))
                    scale_pen = 1.0 / (1.0 + beta * gp)
                    wg = w_base * scale_pen
                    augmented_jacobian = np.vstack([augmented_jacobian, wg * ggc.reshape(1, -1)])
                    augmented_error = np.concatenate([augmented_error, np.array([-wg * gp], dtype=float)])

            delta_q = np.linalg.lstsq(augmented_jacobian, augmented_error, rcond=None)[0]
            step_norm = float(np.linalg.norm(delta_q))
            if step_norm > max_step_rad:
                delta_q = delta_q * (max_step_rad / step_norm)
            q_next = robot.clamp_to_limits(q_next + 0.75 * delta_q)

        q_next = self._refine_q_ground_and_position(robot, q_next, target_position)
        return q_next

    def _refine_q_ground_and_position(
        self,
        robot: SixDofRobot,
        q_init: np.ndarray,
        target_position: np.ndarray,
    ) -> np.ndarray:
        env = robot._environment
        if not bool(env.get("ground_refine_after_ik", True)):
            return np.asarray(q_init, dtype=float).reshape(robot.n)

        floor = robot.joint_z_constraint_floor_m()
        q = np.asarray(q_init, dtype=float).reshape(robot.n).copy()
        max_rounds = int(env.get("ground_refine_max_rounds", 16))
        eta = float(env.get("ground_refine_step", 0.07))
        tcp_fix_iters = int(env.get("ground_refine_tcp_fix_iters", 6))
        target_position = np.asarray(target_position, dtype=float).reshape(3)

        for _ in range(max_rounds):
            if robot.min_checked_joint_height_m(q) > floor + 1e-7:
                break
            gp, g = robot.ground_penalty_grad(q)
            if gp < 1e-14:
                break
            gn = float(np.linalg.norm(g))
            if gn < 1e-12:
                break
            step = eta * float(min(1.0, 0.65 / (np.sqrt(gp + 1e-9))))
            q = robot.clamp_to_limits(q - step * (g / gn))

            for __ in range(tcp_fix_iters):
                cur = robot.fkine(q)[:3, 3]
                err = target_position - cur
                if float(np.linalg.norm(err)) < 1e-5:
                    break
                jac = robot.numerical_jacobian(q)[:3, :]
                try:
                    dq = np.linalg.lstsq(jac, err, rcond=None)[0]
                except np.linalg.LinAlgError:
                    break
                q = robot.clamp_to_limits(q + 0.65 * dq)
        return q

    def _joint_limit_centering_target(self, robot: SixDofRobot, q_rad: np.ndarray) -> np.ndarray:
        lower = robot.qlim_rad[:, 0]
        upper = robot.qlim_rad[:, 1]
        center = 0.5 * (lower + upper)
        half_range = np.maximum(0.5 * (upper - lower), np.deg2rad(1.0))
        normalized = (q_rad - center) / half_range
        # Only push back when the joint has moved into the outer 65% of its range.
        excess = np.maximum(np.abs(normalized) - 0.65, 0.0)
        return -np.sign(normalized) * excess * half_range

    def _axis_vector(self, axis: str) -> np.ndarray:
        axis = axis.lower().strip()
        if axis == "x":
            return np.array([1.0, 0.0, 0.0], dtype=float)
        if axis == "y":
            return np.array([0.0, 1.0, 0.0], dtype=float)
        if axis == "diagonal":
            value = np.array([1.0, 1.0, 1.0], dtype=float)
            return value / np.linalg.norm(value)
        return np.array([0.0, 0.0, 1.0], dtype=float)

    def _orthogonal_basis(self, normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        normal = np.asarray(normal, dtype=float)
        normal = normal / np.linalg.norm(normal)
        arbitrary = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(float(np.dot(arbitrary, normal))) > 0.9:
            arbitrary = np.array([0.0, 1.0, 0.0], dtype=float)
        basis_u = arbitrary - np.dot(arbitrary, normal) * normal
        basis_u = basis_u / np.linalg.norm(basis_u)
        basis_v = np.cross(normal, basis_u)
        basis_v = basis_v / np.linalg.norm(basis_v)
        return basis_u, basis_v
