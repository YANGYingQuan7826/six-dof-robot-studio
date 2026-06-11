from __future__ import annotations

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from qt_app.widgets.plot_style import configure_matplotlib_fonts

configure_matplotlib_fonts()


class JointPlotCanvas(FigureCanvasQTAgg):
    JOINT_TITLES = [
        "q1 底座回转",
        "q2 肩关节",
        "q3 肘关节",
        "q4 腕部 1",
        "q5 腕部 2",
        "q6 末端法兰",
    ]

    def __init__(self, parent=None) -> None:
        self.figure = Figure(figsize=(8, 4.2), facecolor="#ffffff")
        super().__init__(self.figure)
        self.setParent(parent)
        self._draw_placeholder()

    def _draw_placeholder(self) -> None:
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        ax.set_facecolor("#ffffff")
        ax.set_title("关节角变化")
        ax.text(0.2, 0.5, "运行仿真后显示关节轨迹", transform=ax.transAxes, color="#6e6e73")
        ax.axis("off")
        self.draw_idle()

    def update_plot(self, result: dict) -> None:
        time_s = np.asarray(result["time_s"], dtype=float)
        q_deg = np.asarray(result["q_deg"], dtype=float)

        self.figure.clear()
        axes = self.figure.subplots(2, 3, sharex=True)
        axes = np.asarray(axes).reshape(-1)

        colors = ["#007aff", "#34c759", "#ff9500", "#af52de", "#ff2d55", "#5ac8fa"]
        for idx, axis in enumerate(axes):
            if idx >= q_deg.shape[1]:
                axis.axis("off")
                continue

            axis.set_facecolor("#ffffff")
            axis.plot(time_s, q_deg[:, idx], linewidth=1.4, color=colors[idx % len(colors)])
            axis.grid(True, alpha=0.18)
            axis.set_title(self.JOINT_TITLES[idx], fontsize=9, color="#1d1d1f")
            axis.set_ylabel("角度 (deg)")
            axis.tick_params(colors="#6e6e73", labelsize=8)
            axis.yaxis.label.set_color("#424245")
            for spine in axis.spines.values():
                spine.set_color("#e5e5ea")

        for axis in axes[3:]:
            axis.set_xlabel("Time (s)")
            axis.xaxis.label.set_color("#424245")

        self.figure.suptitle("关节角变化：每个小图对应一个机械臂关节", fontsize=11, color="#1d1d1f")
        self.figure.subplots_adjust(left=0.07, right=0.98, top=0.84, bottom=0.14, wspace=0.28, hspace=0.42)
        self.draw_idle()
