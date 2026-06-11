from __future__ import annotations

import copy
import re
from pathlib import Path
from typing import Any

import numpy as np

from python.robot_model import SixDofRobot, default_config_path, load_robot_config
from python.services import LanguageTaskService, RobotProgramService, SimulationService, TaskBlock, TrajectoryCommand, TrajectoryService


class SimulationViewModel:
    def __init__(self, config_path: str | Path | None = None) -> None:
        self.config_path = Path(config_path) if config_path else default_config_path()
        self._config = load_robot_config(self.config_path)
        self._simulation_service = SimulationService()
        self._trajectory_service = TrajectoryService()
        self._language_service = LanguageTaskService()
        self._program_service = RobotProgramService()

    @property
    def config(self) -> dict[str, Any]:
        return copy.deepcopy(self._config)

    def default_rows(self) -> list[dict[str, Any]]:
        return copy.deepcopy(self._config["links"])

    def default_state(self) -> dict[str, Any]:
        return copy.deepcopy(self._config["default_state"])

    def default_simulation(self) -> dict[str, Any]:
        return copy.deepcopy(self._config["simulation_defaults"])

    def build_config(
        self,
        link_rows: list[dict[str, Any]],
        q_init_deg: list[float],
        q_goal_deg: list[float],
        duration_s: float,
        steps: int,
    ) -> dict[str, Any]:
        config = copy.deepcopy(self._config)
        config["links"] = link_rows
        config["default_state"]["q_init_deg"] = q_init_deg
        config["default_state"]["q_goal_deg"] = q_goal_deg
        config["simulation_defaults"]["duration_s"] = float(duration_s)
        config["simulation_defaults"]["steps"] = int(steps)
        return config

    def validate_config(self, config: dict[str, Any]) -> None:
        if len(config["links"]) != 6:
            raise ValueError("当前界面要求 6 个关节参数。")

        for row in config["links"]:
            qlim = row["qlim_deg"]
            if qlim[0] >= qlim[1]:
                raise ValueError(f"{row['joint']} 的关节范围无效。")

        for label, values in {
            "初始关节角": config["default_state"]["q_init_deg"],
            "目标关节角": config["default_state"]["q_goal_deg"],
        }.items():
            if len(values) != 6:
                raise ValueError(f"{label} 必须包含 6 个数值。")

        if config["simulation_defaults"]["steps"] < 5:
            raise ValueError("采样点数至少应为 5。")
        if config["simulation_defaults"]["duration_s"] <= 0:
            raise ValueError("仿真时长必须大于 0。")

    def run_simulation(self, config: dict[str, Any], export_root: str | Path | None = None) -> dict[str, Any]:
        self.validate_config(config)
        results = self._simulation_service.run_default(config, output_root=export_root)
        results["active_config"] = copy.deepcopy(config)
        return results

    def run_trajectory_command(self, config: dict[str, Any], command: TrajectoryCommand) -> dict[str, Any]:
        self.validate_config(config)
        track_result = self._trajectory_service.run_command(config, command)
        return self._wrap_track_result(config, track_result)

    def run_task_blocks(self, config: dict[str, Any], task_blocks: list[TaskBlock]) -> dict[str, Any]:
        self.validate_config(config)
        commands = [task.to_command() for task in task_blocks]
        track_result = self._trajectory_service.run_commands(config, commands)
        track_result["task_blocks"] = [task.__dict__ for task in task_blocks]
        return self._wrap_track_result(config, track_result)

    def precheck_task_blocks(self, config: dict[str, Any], task_blocks: list[TaskBlock]) -> dict[str, Any]:
        self.validate_config(config)
        commands = [task.to_command() for task in task_blocks]
        return self._trajectory_service.precheck_config_commands(config, commands)

    def summarize_precheck(self, precheck: dict[str, Any]) -> str:
        risk_label = {
            "low": "可行",
            "medium": "有风险",
            "high": "不建议直接执行",
        }.get(precheck["risk_level"], precheck["risk_level"])
        lines = [
            f"输入可行性：{risk_label}。目标到最近可达点最大距离 {precheck.get('max_workspace_map_distance_m', 0.0):.3f} m，疑似越界比例 {100 * precheck.get('outside_workspace_map_ratio', 0.0):.1f}%。"
        ]
        auto_start = precheck.get("auto_start")
        if auto_start:
            lines.append(f"自动起点：建议 q_init={auto_start['q_init_deg']}。")
        if precheck.get("ik_preview_passed"):
            preview = (auto_start or {}).get("ik_preview", {})
            lines.append(
                f"IK 试跑：原始尺度可跟踪（最大误差 {float(preview.get('max_error_m', 0.0)):.3f} m），无需按工作空间点云建议缩小半径。"
            )
        suggestions = []
        for segment in precheck.get("segments", []):
            scale = float(segment["recommended_scale"])
            if scale < 0.95:
                recommended_extent = float(segment["requested_extent_m"]) * scale
                if segment["trajectory_type"] == "circle":
                    recommended_radius = recommended_extent / 2.0
                    suggestions.append(f"任务 {segment['index']} 圆半径建议改为 <= {recommended_radius:.3f} m")
                elif segment["trajectory_type"] == "line":
                    suggestions.append(f"任务 {segment['index']} 直线长度建议改为 <= {recommended_extent:.3f} m")
                else:
                    suggestions.append(f"任务 {segment['index']} 建议整体缩小到 {100 * scale:.0f}%")
        if suggestions:
            lines.append("建议：" + "；".join(suggestions))
        elif precheck["risk_level"] == "low":
            lines.append("建议：当前参数可以先生成任务块并执行仿真。")
        return "\n".join(lines)

    def _wrap_track_result(self, config: dict[str, Any], track_result: dict[str, Any]) -> dict[str, Any]:
        ptp_result = copy.deepcopy(track_result)
        ptp_result["summary"] = copy.deepcopy(track_result["summary"])
        ptp_result["summary"]["trajectory_type"] = "command_preview"
        return {
            "fk_demo": self._simulation_service.run_fk(config),
            "ptp": ptp_result,
            "cartesian_track": track_result,
            "active_config": copy.deepcopy(config),
        }

    def parse_task_blocks(self, text: str) -> list[TaskBlock]:
        return self._language_service.parse(text)

    def build_trajectory_command(
        self,
        free_text: str,
        trajectory_type: str,
        axis: str,
        radius_m: float,
        length_m: float,
        rise_m: float,
        turns: float,
        circle_plane_z_m: float | None = None,
    ) -> TrajectoryCommand:
        command = TrajectoryCommand(
            trajectory_type=trajectory_type,
            axis=axis,
            radius_m=float(radius_m),
            length_m=float(length_m),
            rise_m=float(rise_m),
            turns=float(turns),
            circle_plane_z_m=circle_plane_z_m,
        )
        if free_text.strip():
            command = self._parse_trajectory_text(free_text, command)
        return command

    def _parse_trajectory_text(self, text: str, fallback: TrajectoryCommand) -> TrajectoryCommand:
        normalized = text.lower().replace("，", ",")
        trajectory_type = fallback.trajectory_type
        if "直线" in normalized or "line" in normalized:
            trajectory_type = "line"
        elif "螺旋" in normalized or "空间" in normalized or "spatial" in normalized or "helix" in normalized:
            trajectory_type = "spatial_curve"
        elif "圆" in normalized or "circle" in normalized:
            trajectory_type = "circle"

        axis = fallback.axis
        if re.search(r"x\s*y\s*平面|xy\s*plane", normalized):
            axis = "z"
        elif re.search(r"x\s*z\s*平面|xz\s*plane", normalized):
            axis = "y"
        elif re.search(r"y\s*z\s*平面|yz\s*plane", normalized):
            axis = "x"
        elif re.search(r"x\s*轴", normalized) or "x axis" in normalized or "axis x" in normalized:
            axis = "x"
        elif re.search(r"y\s*轴", normalized) or "y axis" in normalized or "axis y" in normalized:
            axis = "y"
        elif re.search(r"z\s*轴", normalized) or "z axis" in normalized or "axis z" in normalized:
            axis = "z"

        radius = self._extract_number(normalized, ["半径", "radius"], fallback.radius_m)
        length = self._extract_number(normalized, ["长度", "直线", "length"], fallback.length_m)
        turns = self._extract_number(normalized, ["圈数", "turns"], fallback.turns)
        if trajectory_type == "spatial_curve":
            rise = self._extract_number(normalized, ["抬升", "高度", "rise"], fallback.rise_m)
        else:
            rise = self._extract_number(normalized, ["抬升", "rise"], fallback.rise_m)
        cz = fallback.circle_plane_z_m
        if trajectory_type == "circle":
            parsed_cz = self._language_service.parse_circle_plane_z(text)
            if parsed_cz is not None:
                cz = parsed_cz
        return TrajectoryCommand(
            trajectory_type=trajectory_type,
            axis=axis,
            radius_m=radius,
            length_m=length,
            rise_m=rise,
            turns=turns,
            circle_plane_z_m=cz,
        )

    def _extract_number(self, text: str, keys: list[str], fallback: float) -> float:
        for key in keys:
            match = re.search(rf"{key}\s*(?:=|为|:)?\s*(-?\d+(?:\.\d+)?)", text)
            if match:
                return float(match.group(1))
        return float(fallback)

    def summarize_results(self, results: dict[str, Any]) -> str:
        fk_demo = results["fk_demo"]["initial_pose"]
        ptp = results["ptp"]
        track = results["cartesian_track"]
        ptp_final = np.asarray(ptp["position_m"])[-1]
        track_final = np.asarray(track["position_m"])[-1]
        assessment = self.assess_results(results)

        return "\n".join(
            [
                f"仿真判定: {assessment['status_text']}",
                f"说明: {assessment['message']}",
                "",
                f"机器人: {results['fk_demo']['robot_name']}",
                f"初始末端位置 (m): {np.round(fk_demo['position_m'], 4).tolist()}",
                f"初始末端姿态 RPY (deg): {np.round(fk_demo['rpy_deg'], 3).tolist()}",
                f"PTP 终点位置 (m): {np.round(ptp_final, 4).tolist()}",
                f"PTP 样本数: {ptp['summary']['samples']}",
                f"轨迹跟踪终点位置 (m): {np.round(track_final, 4).tolist()}",
                f"轨迹跟踪最大位置误差 (m): {track['metrics']['max_position_error_m']:.6f}",
                f"轨迹跟踪平均位置误差 (m): {track['metrics']['mean_position_error_m']:.6f}",
                f"IK 失败点数: {track['metrics'].get('ik_failure_count', 0)}",
                f"近奇异采样点数: {track['metrics'].get('near_singular_count', 0)}",
                f"最大相邻帧关节变化 (deg): {track['metrics'].get('max_joint_step_deg', 0.0):.3f}",
                f"平均相邻帧关节变化 (deg): {track['metrics'].get('mean_joint_step_deg', 0.0):.3f}",
                *self._precheck_lines(track),
                *self._diagnostic_lines(track, assessment),
                *self._segment_diagnostic_lines(track),
                *self._task_summary_lines(track),
                *self._command_summary_lines(track),
                "",
                "图中说明:",
                "蓝色虚线 = 期望轨迹；绿色实线 = 实际末端轨迹；灰色淡线 = 运动过程中的机械臂姿态；黑色粗线 = 最终姿态。",
            ]
        )

    def _precheck_lines(self, track: dict[str, Any]) -> list[str]:
        precheck = track.get("precheck")
        if not precheck:
            return []

        risk_label = {
            "low": "低风险",
            "medium": "中等风险",
            "high": "高风险",
        }.get(precheck["risk_level"], precheck["risk_level"])
        center_shift = precheck["recommended_center_shift_m"]
        q_init = precheck["recommended_q_init_deg"]
        auto_start = precheck.get("auto_start")
        lines = [
            "",
            "轨迹可达性预检查:",
            f"风险等级: {risk_label}",
            f"估计工作半径: {precheck['estimated_workspace_radius_m']:.3f} m；目标最大离基座距离: {precheck['max_requested_distance_from_base_m']:.3f} m；余量: {precheck['workspace_margin_m']:.3f} m",
            f"采样工作空间: {precheck.get('workspace_map_samples', 0)} 个可达点；目标到最近可达点最大距离: {precheck.get('max_workspace_map_distance_m', 0.0):.3f} m；疑似越界比例: {100 * precheck.get('outside_workspace_map_ratio', 0.0):.1f}%",
            f"推荐初始姿态 q_init (deg): {q_init}",
            f"推荐轨迹中心平移 (m): [{center_shift[0]:.3f}, {center_shift[1]:.3f}, {center_shift[2]:.3f}]",
        ]
        if auto_start:
            lines.extend(
                [
                    f"自动选择起点 q_init (deg): {auto_start['q_init_deg']}",
                    f"自动起点末端位置 (m): {auto_start['start_position_m']}",
                    f"选择原因: {auto_start['reason']}",
                ]
            )
            if precheck.get("ik_preview_passed"):
                preview = auto_start.get("ik_preview", {})
                lines.append(
                    f"短轨迹 IK 试跑通过: 最大误差 {float(preview.get('max_error_m', 0.0)):.4f} m，"
                    f"失败率 {100 * float(preview.get('failure_ratio', 0.0)):.1f}%。"
                )

        for segment in precheck.get("segments", []):
            if segment["recommended_scale"] < 0.95:
                lines.append(
                    f"任务 {segment['index']} 建议尺度: 保留约 {100 * segment['recommended_scale']:.0f}% "
                    f"（原始动作尺度约 {segment['requested_extent_m']:.3f} m，"
                    f"最近可达点最大距离 {segment.get('max_workspace_map_distance_m', 0.0):.3f} m）。"
                )
        if precheck["risk_level"] != "low":
            lines.append("建议操作: 先按推荐比例减小任务块参数；若仍误差大，再将 q_init 改为推荐初始姿态。")
        return lines

    def _diagnostic_lines(self, track: dict[str, Any], assessment: dict[str, Any]) -> list[str]:
        metrics = track["metrics"]
        max_error = float(metrics["max_position_error_m"])
        failure_ratio = float(metrics.get("ik_failure_ratio", 0.0))
        near_singular_count = int(metrics.get("near_singular_count", 0))
        max_condition = float(metrics.get("max_jacobian_condition", 0.0))
        was_scaled = self._was_auto_scaled(track)

        lines = ["", "可达性诊断:"]
        if was_scaled:
            lines.append("轨迹尺度偏大：系统已自动缩小半径/长度/抬升后再尝试仿真。建议先减小半径或长度。")
        elif failure_ratio > 0.15 or max_error > 0.02:
            lines.append("轨迹可能超出稳定可达范围：建议减小半径/长度，或把初始姿态调到更靠近轨迹中心的位置。")
        else:
            lines.append("轨迹尺度基本可达：当前机械臂能够较稳定地跟随该任务。")

        if near_singular_count > 0 or max_condition > 200.0:
            lines.append(
                f"接近奇异位形：检测到 {near_singular_count} 个风险采样点，最大雅可比条件数约为 {max_condition:.1f}。建议避开完全伸直/折叠姿态。"
            )
        else:
            lines.append("奇异性风险较低：本次轨迹没有明显接近奇异位形的采样点。")

        if assessment["level"] != "success":
            lines.append("调整建议：优先把圆/螺旋半径减小 30%-50%，其次减少圈数或抬升高度，再重新生成任务块。")
        return lines

    def _segment_diagnostic_lines(self, track: dict[str, Any]) -> list[str]:
        segments = track.get("segment_diagnostics")
        if not segments:
            return []

        lines = ["", "分段诊断:"]
        for segment in segments:
            used = segment["used_command"]
            requested = segment["requested_command"]
            scaled_text = ""
            if segment.get("auto_scaled"):
                scaled_text = (
                    f"，已缩放: 半径 {requested['radius_m']:.3f}->{used['radius_m']:.3f} m，"
                    f"长度 {requested['length_m']:.3f}->{used['length_m']:.3f} m"
                )
            lines.append(
                f"任务 {segment['index']} {segment['trajectory_type']} / {segment['axis']}: "
                f"最大误差 {segment['max_position_error_m']:.4f} m，"
                f"IK 失败率 {100 * segment['ik_failure_ratio']:.1f}%{scaled_text}"
            )
        return lines

    def _was_auto_scaled(self, track: dict[str, Any]) -> bool:
        commands = track.get("commands")
        requested_commands = track.get("requested_commands")
        if commands and requested_commands:
            return any(command != requested for command, requested in zip(commands, requested_commands, strict=False))
        command = track.get("command")
        requested = track.get("requested_command")
        return bool(command and requested and command != requested)

    def _task_summary_lines(self, track: dict[str, Any]) -> list[str]:
        task_blocks = track.get("task_blocks")
        commands = track.get("commands")
        if not task_blocks:
            return []

        lines = ["", f"任务块数量: {len(task_blocks)}"]
        for idx, task in enumerate(task_blocks, start=1):
            source = task.get("source_text") or "手动任务"
            lines.append(f"任务 {idx}: {task['task_type']} / {task['axis']}，来源: {source}")
        if commands:
            lines.append("系统已按任务块顺序拼接为连续仿真轨迹。")
        return lines

    def _command_summary_lines(self, track: dict[str, Any]) -> list[str]:
        command = track.get("command")
        requested = track.get("requested_command")
        if not command:
            return []

        lines = [
            f"执行轨迹: {command['trajectory_type']}，轴/法向: {command['axis']}",
            f"实际使用参数: 半径={command['radius_m']:.4f} m，长度={command['length_m']:.4f} m，抬升={command['rise_m']:.4f} m，圈数={command['turns']:.2f}",
        ]
        if requested and requested != command:
            lines.append(
                f"原始请求参数: 半径={requested['radius_m']:.4f} m，长度={requested['length_m']:.4f} m，抬升={requested['rise_m']:.4f} m"
            )
        return lines

    def assess_results(self, results: dict[str, Any]) -> dict[str, Any]:
        track = results["cartesian_track"]
        max_error = float(track["metrics"]["max_position_error_m"])
        mean_error = float(track["metrics"]["mean_position_error_m"])
        failure_count = int(track["metrics"].get("ik_failure_count", 0))
        failure_ratio = float(track["metrics"].get("ik_failure_ratio", 0.0))
        final_position = np.asarray(track["position_m"], dtype=float)[-1]
        samples = int(track["summary"]["samples"])

        if failure_ratio <= 0.02 and max_error <= 0.005:
            status_text = "模拟成功"
            message = "实际轨迹与期望轨迹吻合较好，可认为当前参数下末端运动可实现。"
            level = "success"
        elif failure_ratio <= 0.15 and max_error <= 0.02:
            status_text = "近似成功"
            message = "轨迹可以跟随，但存在可见误差。建议减小轨迹半径、调整初始姿态或避开奇异位形。"
            level = "warning"
        else:
            status_text = "误差偏大"
            message = "当前目标轨迹较难实现，IK 多次失败或误差过大。建议减小半径/长度，或调整初始姿态。"
            level = "error"

        if self._was_auto_scaled(track):
            message += " 系统已自动缩小轨迹尺度以尝试找到更可达的运动。"
        if int(track["metrics"].get("near_singular_count", 0)) > 0:
            message += " 轨迹中存在接近奇异位形的采样点。"

        return {
            "level": level,
            "status_text": status_text,
            "message": message,
            "max_error_m": max_error,
            "mean_error_m": mean_error,
            "ik_failure_count": failure_count,
            "ik_failure_ratio": failure_ratio,
            "final_position_m": final_position,
            "samples": samples,
        }

    def export_current_results(self, results: dict[str, Any], export_root: str | Path) -> None:
        self._simulation_service.export_results_bundle(results, export_root)

    def export_robot_program(self, results: dict[str, Any], export_root: str | Path) -> Path:
        return self._program_service.export_program(results, export_root)

    def dry_run_send_program(self, results: dict[str, Any]) -> str:
        report = self._program_service.dry_run_send(results)
        return report.message

    def robot_from_config(self, config: dict[str, Any]) -> SixDofRobot:
        return self._simulation_service.robot_from_config(config)
