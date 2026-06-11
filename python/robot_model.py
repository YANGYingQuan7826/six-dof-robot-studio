from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import numpy as np


def workspace_root() -> Path:
    return Path(__file__).resolve().parent.parent


def default_config_path() -> Path:
    return workspace_root() / "config" / "robot_mdh_template.json"


def load_robot_config(config_source: str | Path | dict[str, Any] | None = None) -> dict[str, Any]:
    if config_source is None:
        config_source = default_config_path()

    if isinstance(config_source, dict):
        return copy.deepcopy(config_source)

    path = Path(config_source)
    return json.loads(path.read_text(encoding="utf-8"))


def transform_from_rpy_translation(rpy_deg: list[float], translation_m: list[float]) -> np.ndarray:
    roll, pitch, yaw = np.deg2rad(np.asarray(rpy_deg, dtype=float))
    tx, ty, tz = np.asarray(translation_m, dtype=float)

    rx = np.array(
        [[1.0, 0.0, 0.0], [0.0, np.cos(roll), -np.sin(roll)], [0.0, np.sin(roll), np.cos(roll)]]
    )
    ry = np.array(
        [[np.cos(pitch), 0.0, np.sin(pitch)], [0.0, 1.0, 0.0], [-np.sin(pitch), 0.0, np.cos(pitch)]]
    )
    rz = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    )

    transform = np.eye(4)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = np.array([tx, ty, tz], dtype=float)
    return transform


def mdh_transform(a: float, alpha_rad: float, d: float, theta_rad: float) -> np.ndarray:
    ct = np.cos(theta_rad)
    st = np.sin(theta_rad)
    ca = np.cos(alpha_rad)
    sa = np.sin(alpha_rad)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def standard_dh_transform(a: float, alpha_rad: float, d: float, theta_rad: float) -> np.ndarray:
    """Standard DH: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)."""
    ct = np.cos(theta_rad)
    st = np.sin(theta_rad)
    ca = np.cos(alpha_rad)
    sa = np.sin(alpha_rad)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def homogeneous_rz(theta_rad: float) -> np.ndarray:
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    t = np.eye(4)
    t[0, 0], t[0, 1], t[1, 0], t[1, 1] = c, -s, s, c
    return t


def homogeneous_tx(distance_m: float) -> np.ndarray:
    t = np.eye(4)
    t[0, 3] = float(distance_m)
    return t


def homogeneous_rx(theta_rad: float) -> np.ndarray:
    c, s = np.cos(theta_rad), np.sin(theta_rad)
    t = np.eye(4)
    t[1, 1], t[1, 2], t[2, 1], t[2, 2] = c, -s, s, c
    return t


def rotation_error(current_r: np.ndarray, target_r: np.ndarray) -> np.ndarray:
    return 0.5 * (
        np.cross(current_r[:, 0], target_r[:, 0])
        + np.cross(current_r[:, 1], target_r[:, 1])
        + np.cross(current_r[:, 2], target_r[:, 2])
    )


def rpy_deg_from_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    pitch = np.arctan2(-rotation[2, 0], np.hypot(rotation[2, 1], rotation[2, 2]))
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    return np.rad2deg(np.array([roll, pitch, yaw], dtype=float))


def _length_scale_from_config(config: dict[str, Any]) -> float:
    units = config.get("units") if isinstance(config.get("units"), dict) else {}
    length = str(units.get("length", "m")).strip().lower()
    if length in {"mm", "millimeter", "millimetre"}:
        return 1e-3
    return 1.0


def _translation_vector_m(section: dict[str, Any], length_scale_for_translation_m_field: float) -> np.ndarray:
    if "translation_mm" in section:
        return np.asarray(section["translation_mm"], dtype=float) * 1e-3
    values = np.asarray(section["translation_m"], dtype=float)
    return values * length_scale_for_translation_m_field


def _endpoint_offset_in_frame6_m(config: dict[str, Any]) -> np.ndarray:
    ep = config.get("endpoint7")
    length_scale = _length_scale_from_config(config)
    if not isinstance(ep, dict):
        return np.zeros(3, dtype=float)
    if "offset_in_frame6_mm" in ep:
        return np.asarray(ep["offset_in_frame6_mm"], dtype=float) * 1e-3
    off = np.asarray(ep.get("offset_in_frame6_m", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
    return off * length_scale


def sampled_segment_segment_distance_m(
    a0: np.ndarray,
    a1: np.ndarray,
    b0: np.ndarray,
    b1: np.ndarray,
    samples: int = 10,
) -> float:
    """线段最近距离的采样近似（够用做连杆净空告警）。"""
    ta = np.linspace(0.0, 1.0, samples, dtype=float)
    tb = np.linspace(0.0, 1.0, samples, dtype=float)
    ua = np.asarray([a0 + t * (a1 - a0) for t in ta])
    vb = np.asarray([b0 + t * (b1 - b0) for t in tb])
    dists = np.linalg.norm(ua[:, None, :] - vb[None, :, :], axis=2)
    return float(np.min(dists))


class SixDofRobot:
    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = load_robot_config(config)
        self.links = self.config["links"]
        self.robot_name = self.config["robot_name"]
        self.convention = str(self.config.get("convention", "modified_dh")).strip().lower()
        scale = _length_scale_from_config(self.config)
        # 当单位为 mm 时，translation_m / offset_in_frame6_m 等字段中的数值按毫米解析
        self._length_unit_scale_links = scale
        self.base = transform_from_rpy_translation(
            self.config["base_transform"]["rpy_deg"],
            _translation_vector_m(self.config["base_transform"], scale).tolist(),
        )
        tool_base = transform_from_rpy_translation(
            self.config["tool_transform"]["rpy_deg"],
            _translation_vector_m(self.config["tool_transform"], scale).tolist(),
        )
        off6 = _endpoint_offset_in_frame6_m(self.config)
        if float(np.linalg.norm(off6)) > 0.0:
            flange = np.eye(4, dtype=float)
            flange[:3, 3] = off6
            self.tool = tool_base @ flange
        else:
            self.tool = tool_base

        self.qlim_deg = np.asarray([link["qlim_deg"] for link in self.links], dtype=float)
        self.qlim_rad = np.deg2rad(self.qlim_deg)
        self.offset_rad = np.deg2rad([link["offset_deg"] for link in self.links])
        self.alpha_rad = np.deg2rad([link["alpha_deg"] for link in self.links])
        self.a_m = np.asarray([link["a"] for link in self.links], dtype=float) * scale
        self.d_m = np.asarray([link["d"] for link in self.links], dtype=float) * scale
        self.n = len(self.links)
        cb = self.config.get("compound_base") if isinstance(self.config.get("compound_base"), dict) else {}
        self.use_compound_base: bool = bool(cb.get("enabled", False))
        if "x_offset_mm" in cb:
            self.compound_x_offset_m = float(cb["x_offset_mm"]) * 1e-3
        elif "x_offset_m" in cb:
            self.compound_x_offset_m = float(cb["x_offset_m"]) * scale
        else:
            x_mm = self.links[0].get("x_offset_mm")
            if x_mm is not None:
                self.compound_x_offset_m = float(x_mm) * 1e-3
            elif "x_offset_m" in self.links[0]:
                self.compound_x_offset_m = float(self.links[0]["x_offset_m"]) * scale
            else:
                self.compound_x_offset_m = float(self.a_m[0])

        env = self.config.get("environment")
        env = env if isinstance(env, dict) else {}
        self._environment = env

    def ground_plane_z_m(self) -> float:
        env = self._environment
        if "ground_plane_z_mm" in env:
            return float(env["ground_plane_z_mm"]) * 1e-3
        return float(env.get("ground_plane_z_m", 0.0))

    def joint_z_constraint_floor_m(self) -> float:
        """关节 z 不得低于此高度：`z_ground` + 选配净空(mm)；用于惩罚、预检与硬约束阈值。"""
        z0 = float(self.ground_plane_z_m())
        mm = float(self._environment.get("minimum_joint_clearance_above_ground_mm", 0.0))
        return float(z0 + (mm * 1e-3 if mm > 0.0 else 0.0))

    def joint_indices_above_ground_0based(self) -> list[int]:
        js = self._environment.get("check_joints_1based_above_ground", [2, 3, 4, 5, 6])
        if not isinstance(js, (list, tuple)):
            return list(range(self.n))
        out = [int(j) - 1 for j in js]
        return [k for k in out if 0 <= k < self.n]

    def ground_penalty(self, q_rad: np.ndarray) -> float:
        """低于目标高度则惩罚；mm=0 时目标为地平面 z_ground，推动 z 严格高于地平面。"""
        z_min = self.joint_z_constraint_floor_m()
        strict = float(self._environment.get("ground_penalty_strict_eps_m", 1e-9))
        z_bar = z_min + strict
        origins, _ = self.joint_rotation_axes_world(np.asarray(q_rad, dtype=float).reshape(self.n))
        total = 0.0
        for ji in self.joint_indices_above_ground_0based():
            dz = z_bar - float(origins[ji, 2])
            if dz > 0.0:
                total += dz * dz
        return float(total)

    def ground_penalty_grad(self, q_rad: np.ndarray, eps: float = 5e-4) -> tuple[float, np.ndarray]:
        """地平面违背的平方惩罚 P 及对 q 的数值梯度（用于 IK 附加项）。"""
        q_rad = np.asarray(q_rad, dtype=float).reshape(self.n)
        p0 = self.ground_penalty(q_rad)
        grad = np.zeros(self.n, dtype=float)
        for i in range(self.n):
            dq = np.zeros(self.n, dtype=float)
            dq[i] = eps
            grad[i] = (self.ground_penalty(q_rad + dq) - p0) / eps
        return p0, grad

    def min_checked_joint_height_m(self, q_rad: np.ndarray) -> float:
        """受检关节转轴原点 z 的最小值（世界坐标，米）。"""
        origins, _ = self.joint_rotation_axes_world(np.asarray(q_rad, dtype=float).reshape(self.n))
        idx = self.joint_indices_above_ground_0based()
        if not idx:
            return float("inf")
        return float(min(float(origins[j, 2]) for j in idx))

    def min_checked_joint_clearance_above_floor_m(self, q_rad: np.ndarray) -> float:
        """min(z_checked) − joint_z_constraint_floor_m()；≤0 表示违反「高于地平面」约束。"""
        return float(self.min_checked_joint_height_m(q_rad) - self.joint_z_constraint_floor_m())

    def link_envelope_radius_m(self) -> float:
        r_mm = float(self._environment.get("link_envelope_radius_mm", 0.0))
        return r_mm * 1e-3

    def min_non_adjacent_segment_clearance_m(self, q_rad: np.ndarray) -> float:
        raw = self.min_non_adjacent_segment_separation_raw_m(q_rad)
        env_r = self.link_envelope_radius_m()
        return max(0.0, raw - 2.0 * env_r)

    def min_non_adjacent_segment_separation_raw_m(self, q_rad: np.ndarray) -> float:
        """相邻折线段之间、隔一个及以上索引的线段对的最小距离（未减包络半径）。"""
        pts = self.joint_positions_display(np.asarray(q_rad, dtype=float).reshape(self.n))
        nseg = pts.shape[0] - 1
        if nseg < 2:
            return float("inf")
        min_d = float("inf")
        for i in range(nseg):
            a0, a1 = pts[i], pts[i + 1]
            for j in range(i + 2, nseg):
                b0, b1 = pts[j], pts[j + 1]
                d = sampled_segment_segment_distance_m(a0, a1, b0, b1)
                min_d = min(min_d, d)
        return float(min_d)

    def clamp_to_limits(self, q_rad: np.ndarray) -> np.ndarray:
        q_rad = np.asarray(q_rad, dtype=float)
        return np.clip(q_rad, self.qlim_rad[:, 0], self.qlim_rad[:, 1])

    def fkine(self, q_rad: np.ndarray) -> np.ndarray:
        q_rad = np.asarray(q_rad, dtype=float).reshape(self.n)
        return self._forward_accumulate(q_rad)[1] @ self.tool

    def _forward_accumulate(self, q_rad: np.ndarray) -> tuple[list[np.ndarray], np.ndarray]:
        """返回 (forward_chain frames, cumulative transform without tool)."""
        q_rad = np.asarray(q_rad, dtype=float).reshape(self.n)
        chain: list[np.ndarray] = []

        transform = self.base.copy()
        chain.append(transform.copy())

        if self.use_compound_base:
            q0 = q_rad[0] + self.offset_rad[0]
            transform = transform @ homogeneous_rz(q0)
            transform = transform @ homogeneous_tx(self.compound_x_offset_m)
            chain.append(transform.copy())

            q1 = q_rad[1] + self.offset_rad[1]
            transform = transform @ homogeneous_rx(q1)
            chain.append(transform.copy())

            for idx in range(2, self.n):
                transform = transform @ mdh_transform(
                    self.a_m[idx],
                    self.alpha_rad[idx],
                    self.d_m[idx],
                    q_rad[idx] + self.offset_rad[idx],
                )
                chain.append(transform.copy())
        else:
            for idx in range(self.n):
                dh_fn = standard_dh_transform if self.convention in {"standard_dh", "dh"} else mdh_transform
                transform = transform @ dh_fn(
                    self.a_m[idx],
                    self.alpha_rad[idx],
                    self.d_m[idx],
                    q_rad[idx] + self.offset_rad[idx],
                )
                chain.append(transform.copy())

        return chain, transform

    def forward_chain(self, q_rad: np.ndarray) -> list[np.ndarray]:
        chain, transform = self._forward_accumulate(q_rad)
        chain.append((transform @ self.tool).copy())
        return chain

    def joint_positions(self, q_rad: np.ndarray) -> np.ndarray:
        chain = self.forward_chain(q_rad)
        out: list[np.ndarray] = []
        for i, tf in enumerate(chain):
            if self.use_compound_base and i == 2:
                continue
            p = tf[:3, 3].copy()
            if out and float(np.linalg.norm(p - out[-1])) < 1e-9:
                continue
            out.append(p)
        return np.asarray(out, dtype=float)

    def joint_positions_display(self, q_rad: np.ndarray) -> np.ndarray:
        """与 `joint_positions` 相同（复合底座下 FK 折线已无多余竖直连杆段）。"""
        return self.joint_positions(q_rad)

    def joint_rotation_axes_world(self, q_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """各关节转轴单位方向（世界系）及参考点。复合底座：关节 0 为 z，关节 1 为局部 x。"""
        chain, _ = self._forward_accumulate(q_rad)
        origins = np.zeros((self.n, 3), dtype=float)
        axes = np.zeros((self.n, 3), dtype=float)

        if self.use_compound_base:
            for i in range(self.n):
                t = chain[i]
                o = t[:3, 3]
                r = t[:3, :3]
                if i == 0:
                    v = r[:, 2]
                elif i == 1:
                    v = r[:, 0]
                else:
                    v = r[:, 2]
                n = np.linalg.norm(v)
                if n > 1e-12:
                    v = v / n
                origins[i] = o
                axes[i] = v
            return origins, axes

        for i in range(self.n):
            t = chain[i]
            o = t[:3, 3]
            z = t[:3, 2]
            n = np.linalg.norm(z)
            if n > 1e-12:
                z = z / n
            origins[i] = o
            axes[i] = z
        return origins, axes

    def joint_markers_with_tcp(self, q_rad: np.ndarray) -> np.ndarray:
        """六个关节轴参考点 + TCP，用于球体/标签。"""
        q_rad = np.asarray(q_rad, dtype=float).reshape(self.n)
        origins, _ = self.joint_rotation_axes_world(q_rad)
        tcp = self.fkine(q_rad)[:3, 3].reshape(1, 3)
        return np.vstack([origins, tcp])

    def pose_error(self, q_rad: np.ndarray, target_transform: np.ndarray) -> np.ndarray:
        current = self.fkine(q_rad)
        position_error = target_transform[:3, 3] - current[:3, 3]
        orientation_error = rotation_error(current[:3, :3], target_transform[:3, :3])
        return np.concatenate([position_error, orientation_error])

    def numerical_jacobian(self, q_rad: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        q_rad = np.asarray(q_rad, dtype=float).reshape(self.n)
        base_transform = self.fkine(q_rad)
        base_position = base_transform[:3, 3]
        base_rotation = base_transform[:3, :3]
        jacobian = np.zeros((6, self.n), dtype=float)

        for idx in range(self.n):
            dq = np.zeros_like(q_rad)
            dq[idx] = eps
            perturbed = self.fkine(q_rad + dq)
            jacobian[:3, idx] = (perturbed[:3, 3] - base_position) / eps
            jacobian[3:, idx] = rotation_error(base_rotation, perturbed[:3, :3]) / eps
        return jacobian

    def inverse_kinematics(
        self,
        target_transform: np.ndarray,
        q0_rad: np.ndarray | None = None,
        max_iterations: int | None = None,
        tolerance: float | None = None,
        damping: float = 1e-3,
    ) -> np.ndarray:
        if max_iterations is None:
            max_iterations = int(self.config["simulation_defaults"].get("ik_max_iterations", 200))
        if tolerance is None:
            tolerance = float(self.config["simulation_defaults"].get("ik_tolerance", 1e-5))

        if q0_rad is None:
            q_rad = np.deg2rad(np.asarray(self.config["default_state"]["q_init_deg"], dtype=float))
        else:
            q_rad = np.asarray(q0_rad, dtype=float).reshape(self.n)

        q_rad = self.clamp_to_limits(q_rad)

        for _ in range(max_iterations):
            error = self.pose_error(q_rad, target_transform)
            if np.linalg.norm(error) < tolerance:
                return self.clamp_to_limits(q_rad)

            jacobian = self.numerical_jacobian(q_rad)
            lhs = jacobian @ jacobian.T + (damping**2) * np.eye(6)
            delta_q = jacobian.T @ np.linalg.solve(lhs, error)
            q_rad = self.clamp_to_limits(q_rad + delta_q)

        residual = np.linalg.norm(self.pose_error(q_rad, target_transform))
        if residual < max(2e-3, tolerance * 200.0):
            return self.clamp_to_limits(q_rad)
        raise RuntimeError(f"Inverse kinematics did not converge. Residual={residual:.6e}")

    def inverse_kinematics_position_only(
        self,
        target_position_m: np.ndarray,
        q0_rad: np.ndarray | None = None,
        max_iterations: int | None = None,
        tolerance: float | None = None,
        damping: float = 1e-3,
    ) -> np.ndarray:
        if max_iterations is None:
            max_iterations = int(self.config["simulation_defaults"].get("ik_max_iterations", 400))
        if tolerance is None:
            tolerance = float(self.config["simulation_defaults"].get("ik_tolerance", 1e-5))

        if q0_rad is None:
            q_rad = np.deg2rad(np.asarray(self.config["default_state"]["q_init_deg"], dtype=float))
        else:
            q_rad = np.asarray(q0_rad, dtype=float).reshape(self.n)

        q_rad = self.clamp_to_limits(q_rad)
        target_position_m = np.asarray(target_position_m, dtype=float).reshape(3)

        for _ in range(max_iterations):
            current_transform = self.fkine(q_rad)
            error = target_position_m - current_transform[:3, 3]
            if np.linalg.norm(error) < tolerance:
                return self.clamp_to_limits(q_rad)

            jacobian = self.numerical_jacobian(q_rad)[:3, :]
            lhs = jacobian @ jacobian.T + (damping**2) * np.eye(3)
            delta_q = jacobian.T @ np.linalg.solve(lhs, error)
            q_rad = self.clamp_to_limits(q_rad + delta_q)

        residual = np.linalg.norm(target_position_m - self.fkine(q_rad)[:3, 3])
        if residual < max(5e-3, tolerance * 500.0):
            return self.clamp_to_limits(q_rad)
        raise RuntimeError(f"Position-only IK did not converge. Residual={residual:.6e}")

    def pose_dict(self, q_rad: np.ndarray) -> dict[str, Any]:
        transform = self.fkine(q_rad)
        return {
            "q_deg": np.rad2deg(q_rad).round(6).tolist(),
            "position_m": transform[:3, 3].round(6).tolist(),
            "rpy_deg": rpy_deg_from_transform(transform).round(6).tolist(),
            "transform": transform.round(9).tolist(),
        }
