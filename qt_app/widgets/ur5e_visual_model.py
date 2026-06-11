from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from matplotlib.colors import to_rgb
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from python.robot_model import SixDofRobot, workspace_root


@dataclass(frozen=True)
class MeshBinding:
    name: str
    filename: str
    frame_index: int
    color: str
    alpha: float = 1.0
    local_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    local_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


# Matte light studio palette: unified body + slightly cooler silver on wrist links.
UR5E_COLLISION_BINDINGS: tuple[MeshBinding, ...] = (
    MeshBinding("base", "base.stl", 0, "#e6e8ec", 1.0, local_rpy=(0.0, 0.0, np.pi)),
    MeshBinding("shoulder", "shoulder.stl", 1, "#e8eaee", 1.0, local_rpy=(0.0, 0.0, np.pi)),
    MeshBinding(
        "upperarm",
        "upperarm.stl",
        2,
        "#e4e6ea",
        1.0,
        local_xyz=(0.0, 0.0, 0.138),
        local_rpy=(np.pi / 2.0, 0.0, -np.pi / 2.0),
    ),
    MeshBinding(
        "forearm",
        "forearm.stl",
        3,
        "#e6e8ec",
        1.0,
        local_xyz=(0.0, 0.0, 0.007),
        local_rpy=(np.pi / 2.0, 0.0, -np.pi / 2.0),
    ),
    MeshBinding(
        "wrist1",
        "wrist1.stl",
        4,
        "#e2e4e8",
        1.0,
        local_xyz=(0.0, 0.0, -0.127),
        local_rpy=(np.pi / 2.0, 0.0, 0.0),
    ),
    MeshBinding("wrist2", "wrist2.stl", 5, "#d6d9df", 1.0, local_xyz=(0.0, 0.0, -0.0997)),
    MeshBinding(
        "wrist3",
        "wrist3.stl",
        6,
        "#cdd1d8",
        1.0,
        local_xyz=(0.0, -0.0005, -0.0989),
        local_rpy=(np.pi / 2.0, 0.0, 0.0),
    ),
)


class UR5eVisualModel:
    """Draw downloaded UR5e STL meshes on top of the current FK frames.

    The UR mesh files are not committed to the project. Put collision STL files
    under ``assets/ur5e/collision`` (or use ``scripts/download_ur5e_meshes.py``)
    and this renderer will be enabled automatically. If any required file is
    missing, callers should fall back to the simplified cylinder robot.
    """

    def __init__(self, mesh_dir: str | Path | None = None) -> None:
        self.mesh_dir = Path(mesh_dir) if mesh_dir else workspace_root() / "assets" / "ur5e" / "collision"
        self._tri_cache: dict[Path, np.ndarray] = {}
        self.last_bounds: tuple[np.ndarray, np.ndarray] | None = None
        self.last_frames: list[np.ndarray] = []

    def is_available(self) -> bool:
        return all((self.mesh_dir / binding.filename).exists() for binding in UR5E_COLLISION_BINDINGS)

    def supports(self, robot: SixDofRobot) -> bool:
        visual = robot.config.get("visual_model") if isinstance(robot.config.get("visual_model"), dict) else {}
        return str(visual.get("type", robot.robot_name)).lower() == "ur5e"

    def missing_files(self) -> list[str]:
        return [binding.filename for binding in UR5E_COLLISION_BINDINGS if not (self.mesh_dir / binding.filename).exists()]

    def draw(self, axes, robot: SixDofRobot, q_rad: np.ndarray, label: str | None = None) -> bool:
        if not self.supports(robot) or not self.is_available():
            self.last_bounds = None
            self.last_frames = []
            return False

        frames = self._assembled_link_frames(robot, q_rad)
        self.last_frames = frames
        drew_label = False
        bounds: list[np.ndarray] = []
        for binding in UR5E_COLLISION_BINDINGS:
            if binding.frame_index >= len(frames):
                continue
            triangles = self._load_mesh(self.mesh_dir / binding.filename)
            mesh_tf = frames[binding.frame_index] @ self._tf(list(binding.local_xyz), list(binding.local_rpy))
            world_triangles = self._transform_triangles(triangles, mesh_tf)
            bounds.extend([world_triangles.reshape(-1, 3).min(axis=0), world_triangles.reshape(-1, 3).max(axis=0)])
            facecolors = self._lit_facecolors(world_triangles, binding.color, binding.alpha)
            collection = Poly3DCollection(
                world_triangles,
                facecolors=facecolors,
                edgecolors="none",
                linewidths=0.0,
                antialiaseds=False,
            )
            if label and not drew_label:
                collection.set_label(label)
                drew_label = True
            axes.add_collection3d(collection)
        if bounds:
            bound_arr = np.vstack(bounds)
            self.last_bounds = (bound_arr.min(axis=0), bound_arr.max(axis=0))
        else:
            self.last_bounds = None
        return True

    def link_polyline(self, robot: SixDofRobot, q_rad: np.ndarray) -> np.ndarray:
        frames = self._assembled_link_frames(robot, q_rad)
        tcp = robot.fkine(q_rad)[:3, 3].reshape(1, 3)
        return np.vstack([np.asarray([frame[:3, 3] for frame in frames], dtype=float), tcp])

    def joint_markers_with_tcp(self, robot: SixDofRobot, q_rad: np.ndarray) -> np.ndarray:
        frames = self._assembled_link_frames(robot, q_rad)
        origins = np.asarray([frame[:3, 3] for frame in frames[1:7]], dtype=float)
        tcp = robot.fkine(q_rad)[:3, 3].reshape(1, 3)
        return np.vstack([origins, tcp])

    def joint_rotation_axes_world(self, robot: SixDofRobot, q_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        frames = self._assembled_link_frames(robot, q_rad)
        origins = np.asarray([frame[:3, 3] for frame in frames[1:7]], dtype=float)
        axes = np.asarray([frame[:3, 2] / max(np.linalg.norm(frame[:3, 2]), 1e-12) for frame in frames[1:7]], dtype=float)
        return origins, axes

    def _assembled_link_frames(self, robot: SixDofRobot, q_rad: np.ndarray) -> list[np.ndarray]:
        q = np.asarray(q_rad, dtype=float).reshape(6)
        offset = getattr(robot, "offset_rad", np.zeros(6, dtype=float))
        q_with_offset = q + np.asarray(offset, dtype=float).reshape(6)
        return [robot.base @ frame for frame in self._ur5e_link_frames(q_with_offset)]

    def _load_mesh(self, path: Path) -> np.ndarray:
        cached = self._tri_cache.get(path)
        if cached is not None:
            return cached

        raw = path.read_bytes()
        triangles = self._read_binary_stl(raw)
        if triangles is None:
            triangles = self._read_ascii_stl(raw.decode("utf-8", errors="ignore"))
        self._tri_cache[path] = triangles
        return triangles

    @staticmethod
    def _read_binary_stl(raw: bytes) -> np.ndarray | None:
        if len(raw) < 84:
            return None
        count = struct.unpack_from("<I", raw, 80)[0]
        expected = 84 + count * 50
        if expected != len(raw):
            return None
        triangles = np.zeros((count, 3, 3), dtype=float)
        offset = 84
        for idx in range(count):
            values = struct.unpack_from("<12fH", raw, offset)
            triangles[idx] = np.asarray(values[3:12], dtype=float).reshape(3, 3)
            offset += 50
        return triangles

    @staticmethod
    def _read_ascii_stl(text: str) -> np.ndarray:
        vertices: list[list[float]] = []
        for line in text.splitlines():
            parts = line.strip().split()
            if len(parts) == 4 and parts[0].lower() == "vertex":
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
        if len(vertices) < 3:
            return np.zeros((0, 3, 3), dtype=float)
        usable = len(vertices) - (len(vertices) % 3)
        return np.asarray(vertices[:usable], dtype=float).reshape(-1, 3, 3)

    @staticmethod
    def _transform_triangles(triangles: np.ndarray, transform: np.ndarray) -> np.ndarray:
        rotation = transform[:3, :3]
        translation = transform[:3, 3]
        return triangles @ rotation.T + translation

    @staticmethod
    def _lit_facecolors(triangles: np.ndarray, base_color: str, alpha: float) -> np.ndarray:
        if len(triangles) == 0:
            return np.zeros((0, 4), dtype=float)
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        norm = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = normals / np.maximum(norm, 1e-12)
        light_dir = np.array([0.42, -0.28, 0.86], dtype=float)
        light_dir = light_dir / np.linalg.norm(light_dir)
        view_dir = np.array([0.32, -0.38, 0.87], dtype=float)
        view_dir = view_dir / np.linalg.norm(view_dir)
        half_vec = light_dir + view_dir
        half_vec = half_vec / max(float(np.linalg.norm(half_vec)), 1e-12)
        # STL collision meshes are not guaranteed to have consistent winding.
        # Use two-sided lighting so flipped normals do not appear as black
        # cracks on otherwise valid mesh faces.
        diffuse = np.abs(normals @ light_dir)
        spec = np.clip(np.abs(normals @ half_vec), 0.0, 1.0)
        rim = np.abs(normals @ view_dir)
        # Brushed / matte metal: broad diffuse + narrow specular + soft rim
        shade = 0.64 + 0.22 * diffuse + 0.12 * (spec**3.2) + 0.05 * (rim**2.0)
        shade = np.clip(shade, 0.0, 1.0)
        rgb = np.asarray(to_rgb(base_color), dtype=float)
        rgb = rgb * np.array([0.96, 0.98, 1.02], dtype=float)
        rgb = np.clip(rgb, 0.0, 1.0)
        colors = np.clip(shade[:, None] * rgb[None, :], 0.0, 1.0)
        return np.column_stack([colors, np.full(len(triangles), float(alpha))])

    @classmethod
    def _ur5e_link_frames(cls, q_rad: np.ndarray) -> list[np.ndarray]:
        """UR5e adjacent-part assembly chain from the official default kinematics.

        Each child link frame is obtained by the same parent-child fixed
        transform used by URDF, followed by the revolute joint rotation. This
        keeps neighboring STL parts assembled while the robot moves.
        """
        q = np.asarray(q_rad, dtype=float).reshape(6)
        fixed = [
            cls._tf([0.0, 0.0, 0.1625], [0.0, 0.0, 0.0]),
            cls._tf([0.0, 0.0, 0.0], [np.pi / 2.0, 0.0, 0.0]),
            cls._tf([-0.425, 0.0, 0.0], [0.0, 0.0, 0.0]),
            cls._tf([-0.3922, 0.0, 0.1333], [0.0, 0.0, 0.0]),
            cls._tf([0.0, -0.0997, 0.0], [np.pi / 2.0, 0.0, 0.0]),
            cls._tf([0.0, 0.0996, 0.0], [np.pi / 2.0, np.pi, np.pi]),
        ]

        frames = [np.eye(4)]
        current = np.eye(4)
        for idx, origin_tf in enumerate(fixed):
            current = current @ origin_tf @ cls._rz(q[idx])
            frames.append(current.copy())
        return frames

    @staticmethod
    def _tf(xyz: list[float], rpy: list[float]) -> np.ndarray:
        roll, pitch, yaw = rpy
        rx = np.array(
            [[1.0, 0.0, 0.0], [0.0, np.cos(roll), -np.sin(roll)], [0.0, np.sin(roll), np.cos(roll)]]
        )
        ry = np.array(
            [[np.cos(pitch), 0.0, np.sin(pitch)], [0.0, 1.0, 0.0], [-np.sin(pitch), 0.0, np.cos(pitch)]]
        )
        rz = np.array(
            [[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
        )
        out = np.eye(4)
        out[:3, :3] = rz @ ry @ rx
        out[:3, 3] = np.asarray(xyz, dtype=float)
        return out

    @staticmethod
    def _rz(theta: float) -> np.ndarray:
        c = np.cos(theta)
        s = np.sin(theta)
        out = np.eye(4)
        out[0, 0] = c
        out[0, 1] = -s
        out[1, 0] = s
        out[1, 1] = c
        return out
