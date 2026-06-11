"""Shared trajectory / marker colors for Matplotlib and Qt rich-text legends.

Keep all user-visible trajectory hues in one place so 3D plots and sidebar
legends stay consistent when tuning the studio look.
"""

from __future__ import annotations


class TrajectoryPalette:
    desired = "#7A9CC6"
    actual = "#7FAE90"
    ptp = "#C4A574"
    tcp = "#C4956A"
    start = "#6fad7e"
    end = "#d67b7b"
    hint_gray = "#6b7280"

    @classmethod
    def legend_tracks_rich(cls) -> str:
        return (
            f"<span style='color:{cls.desired}'>●</span> 期望 "
            f"<span style='color:{cls.actual}'>●</span> 实际 "
            f"<span style='color:{cls.ptp}'>●</span> PTP "
            f"<span style='color:{cls.start}'>●</span> 起点 "
            f"<span style='color:{cls.end}'>●</span> 终点"
        )

    @classmethod
    def trajectory_menu_hint_rich(cls) -> str:
        return (
            f"<span style='color:{cls.hint_gray};font-size:11px'>"
            "在菜单「显示」中勾选「显示轨迹」以在右侧画布上显示末端路径。"
            "</span>"
        )
