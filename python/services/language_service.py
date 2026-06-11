from __future__ import annotations

import re
from dataclasses import dataclass

from python.services.trajectory_service import TrajectoryCommand


@dataclass(frozen=True)
class TaskBlock:
    task_type: str
    axis: str = "z"
    radius_m: float = 0.02
    length_m: float = 0.05
    rise_m: float = 0.04
    turns: float = 1.0
    circle_plane_z_m: float | None = None  # draw_circle + axis=z：水平圆锁定世界 z
    source_text: str = ""

    def to_command(self) -> TrajectoryCommand:
        trajectory_type = {
            "move_line": "line",
            "draw_circle": "circle",
            "draw_helix": "spatial_curve",
        }.get(self.task_type, "line")
        return TrajectoryCommand(
            trajectory_type=trajectory_type,
            axis=self.axis,
            radius_m=self.radius_m,
            length_m=self.length_m,
            rise_m=self.rise_m,
            turns=self.turns,
            circle_plane_z_m=self.circle_plane_z_m,
        )


class LanguageTaskService:
    """Rule-based Chinese task parser for offline, deterministic demos."""

    def parse(self, text: str) -> list[TaskBlock]:
        clauses = self._split_clauses(text)
        blocks = [self._parse_clause(clause) for clause in clauses]
        return [block for block in blocks if block is not None]

    def _split_clauses(self, text: str) -> list[str]:
        normalized = text.strip().replace("，", ",").replace("；", ";")
        parts = re.split(r"(?:然后|接着|再|并且|,|;|\n)+", normalized)
        return [part.strip() for part in parts if part.strip()]

    def _parse_clause(self, clause: str) -> TaskBlock | None:
        lower = clause.lower()
        axis = self._parse_axis(lower)
        radius = self._parse_distance(lower, ["半径", "radius"], 0.02)
        length = self._parse_distance(lower, ["长度", "移动", "直线", "length"], 0.05)
        rise = self._parse_distance(lower, ["抬升", "高度", "rise"], 0.04)
        turns = self._parse_number(lower, ["圈数", "圈", "turns"], 1.0)

        if "螺旋" in lower or "空间曲线" in lower or "helix" in lower:
            return TaskBlock("draw_helix", axis=axis, radius_m=radius, length_m=length, rise_m=rise, turns=turns, source_text=clause)
        if "圆" in lower or "circle" in lower:
            circle_z = self._parse_optional_circle_plane_z(lower)
            return TaskBlock(
                "draw_circle",
                axis=axis,
                radius_m=radius,
                length_m=length,
                rise_m=rise,
                turns=turns,
                circle_plane_z_m=circle_z,
                source_text=clause,
            )
        if "直线" in lower or "沿" in lower or "移动" in lower or "line" in lower:
            return TaskBlock("move_line", axis=axis, radius_m=radius, length_m=length, rise_m=rise, turns=turns, source_text=clause)
        return None

    def _parse_axis(self, text: str) -> str:
        if re.search(r"x\s*y\s*平面|xy\s*plane", text):
            return "z"
        if re.search(r"x\s*z\s*平面|xz\s*plane", text):
            return "y"
        if re.search(r"y\s*z\s*平面|yz\s*plane", text):
            return "x"
        if re.search(r"x\s*轴", text) or "x axis" in text or "axis x" in text:
            return "x"
        if re.search(r"y\s*轴", text) or "y axis" in text or "axis y" in text:
            return "y"
        if re.search(r"z\s*轴", text) or "z axis" in text or "axis z" in text:
            return "z"
        if "斜" in text or "对角" in text or "diagonal" in text:
            return "diagonal"
        return "z"

    def _parse_distance(self, text: str, keys: list[str], fallback_m: float) -> float:
        value = self._parse_number(text, keys, fallback_m)
        unit_match = re.search(rf"({'|'.join(keys)})\s*(?:=|为|:)?\s*-?\d+(?:\.\d+)?\s*(毫米|mm|厘米|cm|米|m)?", text)
        unit = unit_match.group(2) if unit_match and unit_match.group(2) else "m"
        if unit in {"毫米", "mm"}:
            return value / 1000.0
        if unit in {"厘米", "cm"}:
            return value / 100.0
        return value

    def _parse_number(self, text: str, keys: list[str], fallback: float) -> float:
        for key in keys:
            match = re.search(rf"{key}\s*(?:=|为|:)?\s*(-?\d+(?:\.\d+)?)", text)
            if match:
                return float(match.group(1))
        return float(fallback)

    def _parse_optional_circle_plane_z(self, text: str) -> float | None:
        """解析水平圆指定高度（世界 z），与螺旋的 rise/高度 分开匹配。"""
        patterns = [
            ("平面z", rf"平面z\s*(?:=|为|:)?\s*(-?\d+(?:\.\d+)?)\s*(毫米|mm|厘米|cm|米|m)?"),
            ("圆z", rf"(?:圆|轨迹)z\s*(?:=|为|:)?\s*(-?\d+(?:\.\d+)?)\s*(毫米|mm|厘米|cm|米|m)?"),
            ("圆高度", rf"圆高度\s*(?:=|为|:)?\s*(-?\d+(?:\.\d+)?)\s*(毫米|mm|厘米|cm|米|m)?"),
            ("高度z", rf"高度z\s*(?:=|为|:)?\s*(-?\d+(?:\.\d+)?)\s*(毫米|mm|厘米|cm|米|m)?"),
            ("circle_z", rf"circle[_\s]*z\s*(?:=|:)?\s*(-?\d+(?:\.\d+)?)\s*(mm|cm|m)?"),
        ]
        for _label, pattern in patterns:
            m = re.search(pattern, text)
            if not m:
                continue
            val = float(m.group(1))
            raw_unit = m.group(2) if len(m.groups()) >= 2 else None
            unit = (raw_unit.strip() if raw_unit else "m").lower()
            if unit in {"毫米", "mm"}:
                val /= 1000.0
            elif unit in {"厘米", "cm"}:
                val /= 100.0
            return val
        return None

    def parse_circle_plane_z(self, text: str) -> float | None:
        """从自然语言中解析「水平圆」的世界 z（米）；未写则返回 None。"""
        return self._parse_optional_circle_plane_z(text.lower())
