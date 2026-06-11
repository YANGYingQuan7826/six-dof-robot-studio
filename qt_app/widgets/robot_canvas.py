from __future__ import annotations

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from python.robot_model import SixDofRobot
from qt_app.widgets.plot_palette import TrajectoryPalette
from qt_app.widgets.plot_style import configure_matplotlib_fonts
from qt_app.widgets.ur5e_visual_model import UR5eVisualModel

configure_matplotlib_fonts()

_STUDIO_BG = "#ECECEE"
_P = TrajectoryPalette


class RobotCanvas(FigureCanvasQTAgg):
    """Matplotlib 3D view with a studio-style default look and optional debug overlays."""

    def __init__(self, parent=None) -> None:
        self.figure = Figure(figsize=(7, 5), facecolor=_STUDIO_BG)
        super().__init__(self.figure)
        self.axes = self.figure.add_subplot(111, projection="3d")
        self.setParent(parent)
        self.visual_model = UR5eVisualModel()
        self.debug_overlays = False
        self.show_trajectories = False
        self._cached_robot: SixDofRobot | None = None
        self._cached_ptp: dict | None = None
        self._cached_track: dict | None = None
        self._preview_q_rad: np.ndarray | None = None
        # Fixed (center, radius) for Cartesian cube limits — set after update_plot, reused in playback.
        self._view_lock: tuple[np.ndarray, float] | None = None
        self._draw_placeholder()

    def set_display_options(self, *, debug_overlays: bool | None = None, show_trajectories: bool | None = None) -> None:
        if debug_overlays is not None:
            self.debug_overlays = bool(debug_overlays)
        if show_trajectories is not None:
            self.show_trajectories = bool(show_trajectories)

    def refresh_display(self) -> None:
        """Re-render from last cached simulation or default preview."""
        if self._cached_robot is None:
            self._draw_placeholder()
            return
        if self._cached_ptp is not None and self._cached_track is not None:
            self.update_plot(self._cached_robot, self._cached_ptp, self._cached_track)
        elif self._preview_q_rad is not None:
            self.show_default_pose(self._cached_robot, self._preview_q_rad)
        else:
            self._draw_placeholder()

    def _draw_placeholder(self) -> None:
        self._view_lock = None
        self.axes.clear()
        self._style_axes()
        self.axes.set_title("")
        self._set_axis_labels_visible(False)
        self.axes.text2D(0.5, 0.5, "加载默认参数后将显示机械臂", transform=self.axes.transAxes, ha="center", color="#9ca3af", fontsize=10)
        self.draw_idle()

    def show_default_pose(self, robot: SixDofRobot, q_rad: np.ndarray) -> None:
        """FK-only preview (no trajectory data)."""
        self._view_lock = None
        self.axes.clear()
        self._style_axes()
        qv = np.asarray(q_rad, dtype=float).reshape(-1)
        joints, markers, axis_origins, axis_dirs = self._display_geometry(robot, qv)

        self._cached_robot = robot
        self._cached_ptp = None
        self._cached_track = None
        self._preview_q_rad = qv.copy()

        if self.debug_overlays:
            self._draw_world_axes()
        visual_bounds = self._draw_visual_or_solid_robot(
            robot,
            qv,
            joints,
            label=None,
            marker_points=markers,
            studio=True,
        )
        if self.debug_overlays:
            self._draw_joint_labels(markers)
            arrow_tips = self._draw_joint_rotation_axes_from(axis_origins, axis_dirs)
        else:
            arrow_tips = np.zeros((0, 3))

        extra: list[np.ndarray] = [joints, markers, arrow_tips]
        if visual_bounds is not None:
            extra.extend(visual_bounds)
        self._fit_camera(np.vstack(extra))
        self.axes.set_title("默认姿态", color="#6b7280", fontsize=10)
        self._set_axis_labels_visible(self.debug_overlays)
        self.axes.view_init(elev=24, azim=-55)
        self.draw_idle()

    def update_plot(self, robot: SixDofRobot, ptp_result: dict, track_result: dict) -> None:
        self.axes.clear()
        self._style_axes()
        self._view_lock = None

        ptp_positions = np.asarray(ptp_result["position_m"], dtype=float)
        track_positions = np.asarray(track_result["position_m"], dtype=float)
        desired_track = np.asarray(track_result["desired_position_m"], dtype=float)
        final_q_rad = np.asarray(track_result["q_rad"], dtype=float)[-1]
        q_track = np.asarray(track_result["q_rad"], dtype=float)
        joints, markers, axis_origins, axis_dirs = self._display_geometry(robot, final_q_rad)

        self._cached_robot = robot
        self._cached_ptp = ptp_result
        self._cached_track = track_result
        self._preview_q_rad = final_q_rad.copy()

        if self.debug_overlays:
            self._draw_world_axes()

        if self.show_trajectories:
            self.axes.plot(
                desired_track[:, 0],
                desired_track[:, 1],
                desired_track[:, 2],
                linestyle="--",
                linewidth=0.9,
                color=_P.desired,
                alpha=0.55,
                label="期望轨迹",
            )
            self.axes.plot(
                track_positions[:, 0],
                track_positions[:, 1],
                track_positions[:, 2],
                linewidth=1.2,
                color=_P.actual,
                alpha=0.65,
                label="实际轨迹",
            )
            self.axes.plot(
                ptp_positions[:, 0],
                ptp_positions[:, 1],
                ptp_positions[:, 2],
                linewidth=0.85,
                alpha=0.45,
                color=_P.ptp,
                label="PTP 末端路径",
            )

        if self.debug_overlays:
            self._draw_motion_sweep(robot, q_track)

        visual_bounds = self._draw_visual_or_solid_robot(
            robot,
            final_q_rad,
            joints,
            label="机械臂",
            marker_points=markers,
            studio=True,
        )
        if self.debug_overlays:
            self._draw_joint_labels(markers)
            arrow_tips = self._draw_joint_rotation_axes_from(axis_origins, axis_dirs)
        else:
            arrow_tips = np.zeros((0, 3))

        if self.show_trajectories:
            self._draw_start_end_points(track_positions)

        # Camera envelope always uses full trajectories so playback zoom matches physics scale.
        extra_points: list[np.ndarray] = [joints, markers, arrow_tips]
        if visual_bounds is not None:
            extra_points.extend(visual_bounds)
        extra_points.extend([ptp_positions, track_positions, desired_track])
        center, radius = self._camera_center_radius(np.vstack(extra_points))
        self._apply_camera_limits(center, radius)
        self._view_lock = (center, radius)

        self.axes.set_title("仿真结果", color="#6b7280", fontsize=10)
        self._set_axis_labels_visible(self.debug_overlays)
        self.axes.view_init(elev=24, azim=-55)
        self.draw_idle()

    def update_frame(self, robot: SixDofRobot, track_result: dict, frame_idx: int) -> None:
        positions = np.asarray(track_result["position_m"], dtype=float)
        desired = np.asarray(track_result["desired_position_m"], dtype=float)
        q_track = np.asarray(track_result["q_rad"], dtype=float)
        frame_idx = int(np.clip(frame_idx, 0, len(q_track) - 1))
        joints, markers, axis_origins, axis_dirs = self._display_geometry(robot, q_track[frame_idx])

        self.axes.clear()
        self._style_axes()

        self._cached_robot = robot
        self._preview_q_rad = q_track[frame_idx].copy()

        if self.debug_overlays:
            self._draw_world_axes()

        # During scrub / playback always show trajectories (independent of menu "显示轨迹").
        self.axes.plot(
            desired[:, 0],
            desired[:, 1],
            desired[:, 2],
            linestyle="--",
            linewidth=0.85,
            color=_P.desired,
            alpha=0.5,
            label="期望轨迹",
        )
        self.axes.plot(
            positions[: frame_idx + 1, 0],
            positions[: frame_idx + 1, 1],
            positions[: frame_idx + 1, 2],
            linewidth=1.1,
            color=_P.actual,
            alpha=0.62,
            label="已完成轨迹",
        )
        self.axes.plot(
            positions[frame_idx:, 0],
            positions[frame_idx:, 1],
            positions[frame_idx:, 2],
            linewidth=0.75,
            color=_P.actual,
            alpha=0.22,
            label="未播放轨迹",
        )

        # Always use full STL / solid model during playback (no polyline substitute).
        visual_bounds = self._draw_visual_or_solid_robot(
            robot,
            q_track[frame_idx],
            joints,
            label="机械臂",
            marker_points=markers,
            studio=True,
        )
        if self.debug_overlays:
            self._draw_joint_labels(markers)
            arrow_tips = self._draw_joint_rotation_axes_from(axis_origins, axis_dirs)
        else:
            arrow_tips = np.zeros((0, 3))

        self._draw_start_end_points(positions)
        current = positions[frame_idx]
        self.axes.scatter(
            current[0],
            current[1],
            current[2],
            color=_P.tcp,
            s=36,
            alpha=0.88,
            label="当前 TCP",
        )

        if self._view_lock is not None:
            c, r = self._view_lock
            self._apply_camera_limits(c, r)
        else:
            extra_points: list[np.ndarray] = [joints, markers, arrow_tips, positions, desired]
            if visual_bounds is not None:
                extra_points.extend(visual_bounds)
            center, radius = self._camera_center_radius(np.vstack(extra_points))
            self._apply_camera_limits(center, radius)
            self._view_lock = (center, radius)

        self.axes.set_title(f"播放 {frame_idx + 1}/{len(q_track)}", color="#6b7280", fontsize=10)
        self._set_axis_labels_visible(self.debug_overlays)
        self.axes.view_init(elev=24, azim=-55)
        self.draw_idle()

    def _camera_center_radius(self, all_points: np.ndarray) -> tuple[np.ndarray, float]:
        mins = all_points.min(axis=0)
        maxs = all_points.max(axis=0)
        center = (mins + maxs) / 2.0
        span = float(np.max(maxs - mins))
        radius = max(0.72 * span / 2.0, 0.14)
        return center, radius

    def _apply_camera_limits(self, center: np.ndarray, radius: float) -> None:
        self.axes.set_xlim(center[0] - radius, center[0] + radius)
        self.axes.set_ylim(center[1] - radius, center[1] + radius)
        self.axes.set_zlim(center[2] - radius, center[2] + radius)

    def _fit_camera(self, all_points: np.ndarray) -> None:
        center, radius = self._camera_center_radius(all_points)
        self._apply_camera_limits(center, radius)

    def _set_axis_labels_visible(self, visible: bool) -> None:
        if visible:
            self.axes.set_xlabel("X (m)")
            self.axes.set_ylabel("Y (m)")
            self.axes.set_zlabel("Z (m)")
            self.axes.tick_params(colors="#86868b", labelsize=8)
        else:
            self.axes.set_xlabel("")
            self.axes.set_ylabel("")
            self.axes.set_zlabel("")
            self.axes.set_xticklabels([])
            self.axes.set_yticklabels([])
            self.axes.set_zticklabels([])

    def _draw_motion_sweep(self, robot: SixDofRobot, q_track: np.ndarray) -> None:
        if len(q_track) < 2:
            return
        sample_count = min(8, len(q_track))
        sample_indices = np.linspace(0, len(q_track) - 1, sample_count, dtype=int)
        for draw_idx, sample_idx in enumerate(sample_indices[:-1]):
            if self.visual_model.supports(robot) and self.visual_model.is_available():
                sample_joints = self.visual_model.link_polyline(robot, q_track[sample_idx])
            else:
                sample_joints = robot.joint_positions_display(q_track[sample_idx])
            alpha = 0.12 + 0.04 * draw_idx
            self.axes.plot(
                sample_joints[:, 0],
                sample_joints[:, 1],
                sample_joints[:, 2],
                color="#8e8e93",
                linewidth=1.0,
                alpha=alpha,
            )

    def _draw_visual_or_solid_robot(
        self,
        robot: SixDofRobot,
        q_rad: np.ndarray,
        joints: np.ndarray,
        label: str | None = None,
        marker_points: np.ndarray | None = None,
        *,
        studio: bool = False,
    ) -> tuple[np.ndarray, np.ndarray] | None:
        show_skeleton = (not studio) or self.debug_overlays
        show_tcp = (not studio) or self.debug_overlays or self.show_trajectories

        if self.visual_model.draw(self.axes, robot, q_rad, label=label):
            m = marker_points if marker_points is not None else joints
            if show_skeleton:
                self.axes.plot(
                    joints[:, 0],
                    joints[:, 1],
                    joints[:, 2],
                    linewidth=0.8,
                    color="#9ca3af",
                    alpha=0.14,
                )
            if show_tcp:
                self._draw_sphere(m[-1], radius=0.015, color=_P.tcp, alpha=0.95)
            return self.visual_model.last_bounds
        self._draw_solid_robot(
            joints,
            label=label,
            marker_points=marker_points,
            show_markers=show_tcp,
            show_backbone=show_skeleton,
        )
        return None

    def _display_geometry(
        self,
        robot: SixDofRobot,
        q_rad: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if self.visual_model.supports(robot) and self.visual_model.is_available():
            joints = self.visual_model.link_polyline(robot, q_rad)
            markers = self.visual_model.joint_markers_with_tcp(robot, q_rad)
            axis_origins, axis_dirs = self.visual_model.joint_rotation_axes_world(robot, q_rad)
            return joints, markers, axis_origins, axis_dirs
        joints = robot.joint_positions_display(q_rad)
        markers = robot.joint_markers_with_tcp(q_rad)
        axis_origins, axis_dirs = robot.joint_rotation_axes_world(q_rad)
        return joints, markers, axis_origins, axis_dirs

    def _draw_solid_robot(
        self,
        joints: np.ndarray,
        label: str | None = None,
        marker_points: np.ndarray | None = None,
        *,
        show_markers: bool = True,
        show_backbone: bool = True,
    ) -> None:
        matte = ["#dcdfe5", "#d8dbe2", "#d4d7de", "#d0d3da", "#cccfd6", "#c8cbd2", "#c4c7ce"]
        for idx in range(len(joints) - 1):
            self._draw_cylinder_between(
                joints[idx],
                joints[idx + 1],
                radius=0.012 if idx < 3 else 0.009,
                color=matte[idx % len(matte)],
                alpha=0.94,
            )

        m = marker_points if marker_points is not None else joints
        if show_markers:
            for idx, point in enumerate(m[:-1]):
                self._draw_sphere(point, radius=0.018 if idx < 4 else 0.014, color="#8b9dc3", alpha=0.55)
            self._draw_sphere(m[-1], radius=0.015, color=_P.tcp, alpha=0.95)

        if show_backbone:
            self.axes.plot(
                joints[:, 0],
                joints[:, 1],
                joints[:, 2],
                linewidth=1.0,
                color="#9ca3af",
                alpha=0.35,
                label=label,
            )

    def _draw_cylinder_between(self, start: np.ndarray, end: np.ndarray, radius: float, color: str, alpha: float) -> None:
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)
        direction = end - start
        length = float(np.linalg.norm(direction))
        if length < 1e-9:
            return
        direction = direction / length
        basis_u, basis_v = self._orthogonal_basis(direction)
        theta = np.linspace(0.0, 2.0 * np.pi, 14)
        z = np.linspace(0.0, length, 2)
        theta_grid, z_grid = np.meshgrid(theta, z)
        circle = radius * (np.cos(theta_grid)[..., None] * basis_u + np.sin(theta_grid)[..., None] * basis_v)
        points = start + z_grid[..., None] * direction + circle
        self.axes.plot_surface(
            points[..., 0],
            points[..., 1],
            points[..., 2],
            color=color,
            alpha=alpha,
            linewidth=0,
            antialiased=True,
            shade=True,
        )

    def _draw_sphere(self, center: np.ndarray, radius: float, color: str, alpha: float) -> None:
        center = np.asarray(center, dtype=float)
        u = np.linspace(0.0, 2.0 * np.pi, 14)
        v = np.linspace(0.0, np.pi, 10)
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
        self.axes.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0, antialiased=True, shade=True)

    def _orthogonal_basis(self, direction: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        direction = np.asarray(direction, dtype=float)
        direction = direction / np.linalg.norm(direction)
        arbitrary = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(arbitrary, direction))) > 0.9:
            arbitrary = np.array([0.0, 1.0, 0.0], dtype=float)
        basis_u = arbitrary - np.dot(arbitrary, direction) * direction
        basis_u = basis_u / np.linalg.norm(basis_u)
        basis_v = np.cross(direction, basis_u)
        basis_v = basis_v / np.linalg.norm(basis_v)
        return basis_u, basis_v

    def _draw_joint_rotation_axes_from(
        self,
        origins: np.ndarray,
        axes: np.ndarray,
        axis_len_m: float = 0.05,
    ) -> np.ndarray:
        palette = ["#5856d6", "#af52de", "#ff2d55", "#ff9500", "#ffd60a", "#30d158"]
        tips = np.zeros_like(origins)
        for i in range(origins.shape[0]):
            o = origins[i]
            dir3 = axes[i]
            n = np.linalg.norm(dir3)
            if n < 1e-12:
                continue
            dir3 = dir3 / n
            delta = axis_len_m * dir3
            tips[i] = o + delta
            self.axes.quiver(
                o[0],
                o[1],
                o[2],
                delta[0],
                delta[1],
                delta[2],
                color=palette[i % len(palette)],
                linewidth=1.8,
                arrow_length_ratio=0.32,
                pivot="tail",
                label="关节轴（+q）" if i == 0 else None,
            )
        return tips

    def _draw_joint_labels(self, joints: np.ndarray) -> None:
        for idx, point in enumerate(joints[:-1]):
            self.axes.text(point[0], point[1], point[2], f"J{idx}", color="#374151", fontsize=8)
        tip = joints[-1]
        self.axes.text(tip[0], tip[1], tip[2], "TCP", color="#2563eb", fontsize=9)

    def _draw_start_end_points(self, track_positions: np.ndarray) -> None:
        start = track_positions[0]
        end = track_positions[-1]
        self.axes.scatter(start[0], start[1], start[2], color=_P.start, s=32, alpha=0.75, label="起点")
        self.axes.scatter(end[0], end[1], end[2], color=_P.end, s=38, alpha=0.75, label="终点")

    def _draw_world_axes(self) -> None:
        axis_len = 0.12
        origin = np.zeros(3)
        self.axes.quiver(*origin, axis_len, 0, 0, color="#e11d48", linewidth=1.2)
        self.axes.quiver(*origin, 0, axis_len, 0, color="#16a34a", linewidth=1.2)
        self.axes.quiver(*origin, 0, 0, axis_len, color="#2563eb", linewidth=1.2)
        self.axes.text(axis_len, 0, 0, "X", color="#e11d48", fontsize=9)
        self.axes.text(0, axis_len, 0, "Y", color="#16a34a", fontsize=9)
        self.axes.text(0, 0, axis_len, "Z", color="#2563eb", fontsize=9)

    def _style_axes(self) -> None:
        self.figure.patch.set_facecolor(_STUDIO_BG)
        self.figure.subplots_adjust(left=0.02, right=0.98, top=0.94, bottom=0.02)
        self.axes.set_facecolor(_STUDIO_BG)
        self.axes.grid(False)
        for axis in (self.axes.xaxis, self.axes.yaxis, self.axes.zaxis):
            axis.pane.set_facecolor(_STUDIO_BG)
            axis.pane.set_edgecolor(_STUDIO_BG)
            axis.pane.set_alpha(1.0)
        self.axes.xaxis.label.set_color("#4b5563")
        self.axes.yaxis.label.set_color("#4b5563")
        self.axes.zaxis.label.set_color("#4b5563")
        self.axes.title.set_color("#6b7280")
