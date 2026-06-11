from __future__ import annotations

import logging
from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QGraphicsDropShadowEffect,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QSlider,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from python.services import TaskBlock
from qt_app.viewmodels.simulation_viewmodel import SimulationViewModel
from qt_app.widgets.joint_plot_canvas import JointPlotCanvas
from qt_app.widgets.plot_palette import TrajectoryPalette
from qt_app.widgets.robot_canvas import RobotCanvas

_LOG = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    TABLE_COLUMNS = [
        ("joint", "Joint"),
        ("name", "Name"),
        ("a", "a (m)"),
        ("alpha_deg", "alpha (deg)"),
        ("d", "d (m)"),
        ("offset_deg", "offset (deg)"),
        ("qlim_min", "Q0 (deg)"),
        ("qlim_max", "Q1 (deg)"),
    ]

    def __init__(self, viewmodel: SimulationViewModel) -> None:
        super().__init__()
        self.viewmodel = viewmodel
        self.current_results = None
        self.current_robot = None
        self.current_task_blocks = []
        self.animation_result = None
        self.animation_frame = 0
        self.animation_timer = QTimer(self)
        # Matplotlib 3D redraws are CPU-heavy; a lower frame rate keeps the UI responsive.
        self.animation_timer.setInterval(80)
        self.animation_timer.timeout.connect(self._advance_animation)
        self.feasibility_timer = QTimer(self)
        self.feasibility_timer.setSingleShot(True)
        self.feasibility_timer.setInterval(700)
        self.feasibility_timer.timeout.connect(self._update_input_feasibility)
        self._preview_timer = QTimer(self)
        self._preview_timer.setSingleShot(True)
        self._preview_timer.setInterval(350)
        self._preview_timer.timeout.connect(self._refresh_robot_preview)
        self._settings_dialog: QDialog | None = None

        self.setWindowTitle("六自由度机械臂运动学仿真")
        self.resize(1500, 900)
        self.setMinimumSize(1180, 760)

        self._build_ui()
        self._load_defaults()

    def _build_ui(self) -> None:
        self._build_menu_bar()

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        root_layout = QHBoxLayout(central_widget)
        root_layout.setContentsMargins(20, 16, 20, 16)
        root_layout.setSpacing(16)

        splitter = QSplitter(Qt.Horizontal)
        root_layout.addWidget(splitter)

        self.parameter_group = self._build_parameter_group()
        self.simulation_group = self._build_simulation_group()
        self.guidance_group = self._build_guidance_group()
        self._settings_host = QWidget(self)
        self._settings_host.hide()
        for group in (self.parameter_group, self.simulation_group, self.guidance_group):
            group.setParent(self._settings_host)

        left_panel = QFrame()
        left_panel.setObjectName("LeftColumn")
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(14, 14, 14, 14)
        left_layout.setSpacing(16)

        left_layout.addWidget(self._build_top_bar())
        left_layout.addWidget(self._build_trajectory_command_card())
        left_layout.addWidget(self._build_joint_plot_card())
        left_layout.addWidget(self._build_result_overview_card())
        left_layout.addWidget(self._build_result_text_card(), stretch=1)
        self._apply_left_panel_shadow(left_panel)

        right_panel = QWidget()
        right_panel.setObjectName("RightColumn")
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(10, 0, 0, 0)
        right_layout.setSpacing(12)
        self.robot_canvas = RobotCanvas()
        right_layout.addWidget(self.robot_canvas, stretch=1)
        right_layout.addWidget(self._build_animation_strip())

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([520, 980])

        self.statusBar().showMessage("已加载默认模型参数。")

    @staticmethod
    def _apply_left_panel_shadow(panel: QFrame) -> None:
        shadow = QGraphicsDropShadowEffect(panel)
        shadow.setBlurRadius(18)
        shadow.setOffset(0, 3)
        shadow.setColor(QColor(15, 23, 42, 26))
        panel.setGraphicsEffect(shadow)

    def _build_menu_bar(self) -> None:
        view_menu = self.menuBar().addMenu("显示")
        self._action_debug = QAction("调试叠加层", self, checkable=True)
        self._action_debug.setChecked(False)
        self._action_trajectories = QAction("显示轨迹", self, checkable=True)
        self._action_trajectories.setChecked(False)
        view_menu.addAction(self._action_debug)
        view_menu.addAction(self._action_trajectories)
        self._action_debug.toggled.connect(self._on_debug_toggled)
        self._action_trajectories.toggled.connect(self._on_trajectories_toggled)

    def _on_debug_toggled(self, checked: bool) -> None:
        self.robot_canvas.set_display_options(debug_overlays=checked)
        self._redraw_robot_canvas()

    def _on_trajectories_toggled(self, checked: bool) -> None:
        self.robot_canvas.set_display_options(show_trajectories=checked)
        self._update_legend_strip()
        self._redraw_robot_canvas()

    def _redraw_robot_canvas(self) -> None:
        if self.current_robot is None:
            self._refresh_robot_preview()
            return
        if self.animation_timer.isActive() and self.animation_result is not None:
            self._draw_animation_frame(self.animation_frame)
            return
        if self.current_results is not None:
            self.robot_canvas.update_plot(
                self.current_robot,
                self.current_results["ptp"],
                self.current_results["cartesian_track"],
            )
            return
        self._refresh_robot_preview()

    def _open_settings(self) -> None:
        dialog = self._ensure_settings_dialog()
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

    def _ensure_settings_dialog(self) -> QDialog:
        if self._settings_dialog is not None:
            return self._settings_dialog
        dialog = QDialog(self)
        dialog.setWindowTitle("设置")
        dialog.setMinimumWidth(760)
        dialog.setMinimumHeight(560)
        dialog.setAttribute(Qt.WA_DeleteOnClose, False)
        dialog.setWindowModality(Qt.NonModal)
        layout = QVBoxLayout(dialog)
        tabs = QTabWidget()
        tabs.addTab(self.parameter_group, "MDH 参数")
        tabs.addTab(self.guidance_group, "调参说明")
        tabs.addTab(self.simulation_group, "仿真设置")
        layout.addWidget(tabs)
        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(dialog.reject)
        dialog.finished.connect(lambda _r: self._schedule_robot_preview_refresh())
        layout.addWidget(buttons)
        self._settings_dialog = dialog
        return dialog

    def _build_top_bar(self) -> QWidget:
        bar = QWidget()
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        title_block = QVBoxLayout()
        title_block.setSpacing(2)
        title = QLabel("6-DOF Robot Studio")
        title.setObjectName("TopBarTitle")
        subtitle = QLabel("运动学仿真 · 轨迹 · 可视化")
        subtitle.setObjectName("TopBarSubtitle")
        title_block.addWidget(title)
        title_block.addWidget(subtitle)
        layout.addLayout(title_block)
        layout.addStretch(1)

        settings_btn = QPushButton("⚙ 设置")
        settings_btn.setObjectName("GhostButton")
        settings_btn.setToolTip("MDH 参数、仿真设置与调参说明")
        settings_btn.clicked.connect(self._open_settings)
        layout.addWidget(settings_btn, alignment=Qt.AlignTop)
        return bar

    def _build_trajectory_command_card(self) -> QFrame:
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setSpacing(12)

        self.command_edit = QLineEdit()
        self.command_edit.setObjectName("CommandInput")
        self.command_edit.setPlaceholderText("输入轨迹：沿 x 轴移动 长度=0.05m / 在 xy 平面画圆 半径=0.08m 平面z=50mm")
        self.command_edit.textChanged.connect(self._schedule_feasibility_check)

        command_row = QHBoxLayout()
        command_row.setSpacing(10)
        command_row.addWidget(self.command_edit, stretch=1)

        self.run_command_button = QPushButton("▶")
        self.run_command_button.setObjectName("PlayButton")
        self.run_command_button.setToolTip("生成并执行轨迹任务")
        self.run_command_button.clicked.connect(self._run_trajectory_input)
        command_row.addWidget(self.run_command_button)

        self.trajectory_more_button = QPushButton("⋯")
        self.trajectory_more_button.setObjectName("IconButton")
        self.trajectory_more_button.setToolTip("显示/隐藏高级轨迹参数")
        self.trajectory_more_button.clicked.connect(self._toggle_trajectory_tools)
        command_row.addWidget(self.trajectory_more_button)
        layout.addLayout(command_row)

        self.feasibility_label = QLabel("输入可行性：请输入自然语言任务后自动判断。")
        self.feasibility_label.setObjectName("GuideLabel")
        self.feasibility_label.setWordWrap(True)
        layout.addWidget(self.feasibility_label)

        self.trajectory_tools_panel = QWidget()
        self.trajectory_tools_panel.setObjectName("AdvancedPanel")
        tools_layout = QVBoxLayout(self.trajectory_tools_panel)
        tools_layout.setContentsMargins(12, 12, 12, 12)
        tools_layout.setSpacing(10)

        self.trajectory_type_combo = QComboBox()
        self.trajectory_type_combo.addItems(["line", "circle", "spatial_curve"])
        self.axis_combo = QComboBox()
        self.axis_combo.addItems(["x", "y", "z", "diagonal"])
        self.radius_edit = QLineEdit("0.025")
        self.length_edit = QLineEdit("0.05")
        self.rise_edit = QLineEdit("0.04")
        self.turns_edit = QLineEdit("1.0")
        self.circle_plane_z_edit = QLineEdit("")
        self.circle_plane_z_edit.setPlaceholderText("仅 circle+z：留空=随起点")

        param_grid = QGridLayout()
        param_grid.setHorizontalSpacing(10)
        param_grid.setVerticalSpacing(8)
        self._add_compact_param(param_grid, 0, 0, "轨迹类型", self.trajectory_type_combo)
        self._add_compact_param(param_grid, 0, 1, "轴/法向", self.axis_combo)
        self._add_compact_param(param_grid, 0, 2, "半径 (m)", self.radius_edit)
        self._add_compact_param(param_grid, 1, 0, "长度 (m)", self.length_edit)
        self._add_compact_param(param_grid, 1, 1, "抬升 (m)", self.rise_edit)
        self._add_compact_param(param_grid, 1, 2, "圈数", self.turns_edit)
        self._add_compact_param(param_grid, 2, 0, "圆平面高度 z (m)", self.circle_plane_z_edit)
        self.trajectory_type_combo.currentTextChanged.connect(self._on_trajectory_command_type_changed)
        self.axis_combo.currentTextChanged.connect(self._on_trajectory_axis_for_circle_plane)
        tools_layout.addLayout(param_grid)

        self._on_trajectory_axis_for_circle_plane()

        self.generate_tasks_button = QPushButton("生成任务块")
        self.run_tasks_button = QPushButton("执行任务块")
        self.generate_tasks_button.hide()
        self.run_tasks_button.hide()
        self.generate_tasks_button.clicked.connect(self._generate_task_blocks)
        self.run_tasks_button.clicked.connect(self._run_task_blocks)

        self.move_task_up_button = QPushButton("上移")
        self.move_task_down_button = QPushButton("下移")
        self.delete_task_button = QPushButton("删除选中")
        self.move_task_up_button.hide()
        self.move_task_down_button.hide()
        self.delete_task_button.hide()
        self.move_task_up_button.clicked.connect(self._move_selected_task_up)
        self.move_task_down_button.clicked.connect(self._move_selected_task_down)
        self.delete_task_button.clicked.connect(self._delete_selected_tasks)

        self.task_table = QTableWidget(0, 8)
        self.task_table.setHorizontalHeaderLabels(["类型", "轴/法向", "半径", "长度", "抬升", "圈数", "圆平面z(m)", "原始文本"])
        self.task_table.verticalHeader().setVisible(False)
        self.task_table.horizontalHeader().setStretchLastSection(True)
        self.task_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.task_table.setMaximumHeight(150)
        self.task_table.hide()
        tools_layout.addWidget(self.task_table)
        self.trajectory_tools_panel.hide()
        layout.addWidget(self.trajectory_tools_panel)

        card = QFrame()
        card.setObjectName("AppCard")
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(18, 16, 18, 16)
        card_layout.setSpacing(12)
        section_title = QLabel("轨迹输入")
        section_title.setObjectName("SectionTitle")
        card_layout.addWidget(section_title)
        card_layout.addWidget(inner)
        return card

    def _toggle_trajectory_tools(self) -> None:
        if not hasattr(self, "trajectory_tools_panel"):
            return
        self.trajectory_tools_panel.setVisible(not self.trajectory_tools_panel.isVisible())

    def _run_trajectory_input(self) -> None:
        text = self.command_edit.text().strip()
        if text:
            try:
                task_blocks = self.viewmodel.parse_task_blocks(text)
            except Exception:
                task_blocks = []
            if task_blocks:
                self.current_task_blocks = task_blocks
                self._render_task_blocks(task_blocks)
                self._run_task_blocks()
                return
        self._run_trajectory_command()

    def _read_optional_circle_plane_z(self) -> float | None:
        text = self.circle_plane_z_edit.text().strip()
        if not text or text.lower() in {"", "none", "-"}:
            return None
        return float(text)

    def _on_trajectory_command_type_changed(self, trajectory_type: str) -> None:
        self._on_trajectory_axis_for_circle_plane()

    def _on_trajectory_axis_for_circle_plane(self) -> None:
        traj = self.trajectory_type_combo.currentText().lower().strip()
        axis = self.axis_combo.currentText().strip().lower()
        use_height = traj == "circle" and axis == "z"
        self.circle_plane_z_edit.setEnabled(use_height)
        self.circle_plane_z_edit.setPlaceholderText("留空=随起点高度" if use_height else "仅轨迹 circle 且轴线 z 时启用")

    def _add_compact_param(self, layout: QGridLayout, row: int, col: int, label_text: str, widget: QWidget) -> None:
        cell = QWidget()
        cell_layout = QVBoxLayout(cell)
        cell_layout.setContentsMargins(0, 0, 0, 0)
        cell_layout.setSpacing(4)
        label = QLabel(label_text)
        label.setObjectName("MetricCaption")
        cell_layout.addWidget(label)
        cell_layout.addWidget(widget)
        layout.addWidget(cell, row, col)

    def _build_parameter_group(self) -> QGroupBox:
        group = QGroupBox("MDH 参数与关节范围")
        layout = QVBoxLayout(group)
        layout.setSpacing(10)

        self.parameter_table = QTableWidget(6, len(self.TABLE_COLUMNS))
        self.parameter_table.setHorizontalHeaderLabels([label for _, label in self.TABLE_COLUMNS])
        self.parameter_table.verticalHeader().setVisible(False)
        self.parameter_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.parameter_table.horizontalHeader().setStretchLastSection(True)
        self.parameter_table.setAlternatingRowColors(True)
        self.parameter_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.parameter_table.setMinimumHeight(280)
        hint_label = QLabel("可手动修改 a、alpha、d、offset 和关节范围。")
        hint_label.setObjectName("SubtitleLabel")
        layout.addWidget(hint_label)
        layout.addWidget(self.parameter_table)
        self.parameter_table.itemChanged.connect(self._on_parameter_table_changed)

        return group

    def _build_simulation_group(self) -> QGroupBox:
        group = QGroupBox("仿真设置")
        layout = QVBoxLayout(group)
        layout.setSpacing(12)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)
        form.setFormAlignment(Qt.AlignTop)
        form.setHorizontalSpacing(16)
        form.setVerticalSpacing(10)
        self.q_init_edit = QLineEdit()
        self.q_goal_edit = QLineEdit()
        self.duration_edit = QLineEdit()
        self.steps_edit = QLineEdit()
        self.q_init_edit.setPlaceholderText("例如: 20, -35, 75, 15, 35, -20")
        self.q_goal_edit.setPlaceholderText("例如: -25, -5, 55, 40, 20, 35")
        self.duration_edit.setPlaceholderText("仿真持续时间")
        self.steps_edit.setPlaceholderText("采样越多曲线越平滑")

        form.addRow("初始关节角 q_init (deg)", self.q_init_edit)
        form.addRow("目标关节角 q_goal (deg)", self.q_goal_edit)
        form.addRow("仿真时长 (s)", self.duration_edit)
        form.addRow("采样点数", self.steps_edit)
        layout.addLayout(form)

        self.q_init_edit.textChanged.connect(self._schedule_robot_preview_refresh)

        button_row = QHBoxLayout()
        self.reset_button = QPushButton("恢复默认参数")
        self.run_button = QPushButton("运行仿真")
        self.export_button = QPushButton("导出当前结果")
        self.run_button.setObjectName("PrimaryButton")
        button_row.addWidget(self.reset_button)
        button_row.addWidget(self.run_button)
        button_row.addWidget(self.export_button)
        layout.addLayout(button_row)

        self.clear_results_preview_button = QPushButton("清空结果并预览")
        self.clear_results_preview_button.setToolTip("清除仿真结果与动画缓存，按当前 MDH 与 q_init 在右侧显示预览")
        self.clear_results_preview_button.clicked.connect(self._clear_results_and_preview)
        layout.addWidget(self.clear_results_preview_button)

        self.reset_button.clicked.connect(self._load_defaults)
        self.run_button.clicked.connect(self._run_simulation)
        self.export_button.clicked.connect(self._export_results)
        return group

    def _build_animation_strip(self) -> QFrame:
        strip = QFrame()
        strip.setObjectName("ToolStrip")
        controls = QHBoxLayout(strip)
        controls.setContentsMargins(14, 10, 14, 10)
        controls.setSpacing(10)

        self.play_button = QPushButton("▶")
        self.pause_button = QPushButton("Ⅱ")
        self.reset_animation_button = QPushButton("↺")
        self.export_program_button = QPushButton("⇩")
        self.dry_run_send_button = QPushButton("⇢")
        for button, tip in (
            (self.play_button, "播放"),
            (self.pause_button, "暂停"),
            (self.reset_animation_button, "回到起点"),
            (self.export_program_button, "导出机器人程序"),
            (self.dry_run_send_button, "虚拟下发"),
        ):
            button.setObjectName("IconButton")
            button.setToolTip(tip)
        self.frame_slider = QSlider(Qt.Horizontal)
        self.frame_slider.setMinimum(0)
        self.frame_slider.setMaximum(0)
        self.frame_slider.valueChanged.connect(self._seek_animation)

        self.play_button.clicked.connect(self._play_animation)
        self.pause_button.clicked.connect(self._pause_animation)
        self.reset_animation_button.clicked.connect(self._reset_animation)
        self.export_program_button.clicked.connect(self._export_robot_program)
        self.dry_run_send_button.clicked.connect(self._dry_run_send_program)

        controls.addWidget(self.play_button)
        controls.addWidget(self.pause_button)
        controls.addWidget(self.reset_animation_button)
        controls.addWidget(self.export_program_button)
        controls.addWidget(self.dry_run_send_button)
        controls.addWidget(self.frame_slider, stretch=1)

        self.legend_strip = QLabel()
        self.legend_strip.setObjectName("LegendStrip")
        self.legend_strip.setWordWrap(True)
        self.legend_strip.setTextFormat(Qt.TextFormat.RichText)
        self.legend_strip.hide()

        return strip

    def _build_joint_plot_card(self) -> QFrame:
        self.joint_canvas = JointPlotCanvas()
        card = QFrame()
        card.setObjectName("AppCard")
        lay = QVBoxLayout(card)
        lay.setContentsMargins(16, 14, 16, 14)
        lay.setSpacing(10)
        title = QLabel("闪电关节角变化")
        title.setObjectName("SectionTitle")
        subtitle = QLabel("每个小图对应一个机械臂关节")
        subtitle.setObjectName("SubtitleLabel")
        lay.addWidget(title)
        lay.addWidget(subtitle)
        self.joint_canvas.setMinimumHeight(220)
        lay.addWidget(self.joint_canvas, stretch=1)
        return card

    def _build_result_text_card(self) -> QFrame:
        self.result_box = QPlainTextEdit()
        self.result_box.setReadOnly(True)
        self.result_box.setPlaceholderText("运行仿真后在这里显示末端位姿和误差统计。")
        self.result_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.result_box.setMinimumHeight(140)

        card = QFrame()
        card.setObjectName("AppCard")
        lay = QVBoxLayout(card)
        lay.setContentsMargins(16, 14, 16, 14)
        lay.setSpacing(8)
        title = QLabel("输出")
        title.setObjectName("SectionTitle")
        lay.addWidget(title)
        lay.addWidget(self.result_box, stretch=1)
        return card

    def _build_guidance_group(self) -> QGroupBox:
        group = QGroupBox("调参提示")
        layout = QVBoxLayout(group)
        layout.setSpacing(8)

        guide_text = (
            "a：主要影响相邻关节轴之间的横向距离，通常会改变机械臂可达半径。\n"
            "d：主要影响沿关节轴方向的偏置，会改变高度或腕部伸出量。\n"
            "alpha：改变相邻关节轴的夹角，调错会显著改变机械臂形态。\n"
            "q_init / q_goal：决定运动从哪里开始、到哪里结束；若关节接近极限或完全伸直，容易出现误差或奇异。"
        )
        guide = QLabel(guide_text)
        guide.setObjectName("GuideLabel")
        guide.setWordWrap(True)
        layout.addWidget(guide)
        return group

    def _build_result_overview_card(self) -> QFrame:
        inner = QWidget()
        layout = QGridLayout(inner)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(10)

        self.status_pill = QLabel("未运行")
        self.status_pill.setObjectName("StatusPill")
        self.status_message_label = QLabel("在设置中配置仿真参数后运行仿真，这里会显示是否成功。")
        self.status_message_label.setObjectName("SubtitleLabel")
        self.status_message_label.setWordWrap(True)

        layout.addWidget(self.status_pill, 0, 0, 1, 1)
        layout.addWidget(self.status_message_label, 0, 1, 1, 3)

        self.max_error_card, self.max_error_value = self._create_metric_card("最大误差", "-- m")
        self.mean_error_card, self.mean_error_value = self._create_metric_card("平均误差", "-- m")
        self.ik_failure_card, self.ik_failure_value = self._create_metric_card("IK 失败率", "--")
        self.final_position_card, self.final_position_value = self._create_metric_card("末端位置", "--")

        layout.addWidget(self.max_error_card, 1, 0)
        layout.addWidget(self.mean_error_card, 1, 1)
        layout.addWidget(self.ik_failure_card, 1, 2)
        layout.addWidget(self.final_position_card, 1, 3)

        card = QFrame()
        card.setObjectName("AppCard")
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(18, 16, 18, 16)
        card_layout.setSpacing(12)
        section_title = QLabel("仿真状态")
        section_title.setObjectName("SectionTitle")
        card_layout.addWidget(section_title)
        card_layout.addWidget(inner)
        return card

    def _create_metric_card(self, caption: str, value: str) -> tuple[QFrame, QLabel]:
        card = QFrame()
        card.setObjectName("MetricCard")
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(12, 10, 12, 10)
        card_layout.setSpacing(4)

        caption_label = QLabel(caption)
        caption_label.setObjectName("MetricCaption")
        value_label = QLabel(value)
        value_label.setObjectName("MetricValue")
        value_label.setWordWrap(True)

        card_layout.addWidget(caption_label)
        card_layout.addWidget(value_label)
        return card, value_label

    def _on_parameter_table_changed(self, _item: QTableWidgetItem) -> None:
        if self.current_results is not None:
            self._invalidate_simulation_results(
                clear_text=True,
                message="已根据新的 MDH 参数清空仿真结果，右侧为预览姿态。",
            )
            return
        self._schedule_robot_preview_refresh()

    def _clear_results_and_preview(self) -> None:
        self._invalidate_simulation_results(
            clear_text=True,
            message="已清空仿真结果，右侧为预览姿态。",
        )

    def _invalidate_simulation_results(self, *, clear_text: bool, message: str) -> None:
        self.animation_timer.stop()
        self.animation_result = None
        self.current_results = None
        self.animation_frame = 0
        if hasattr(self, "frame_slider"):
            self.frame_slider.blockSignals(True)
            self.frame_slider.setMaximum(0)
            self.frame_slider.setValue(0)
            self.frame_slider.blockSignals(False)
        if clear_text:
            self.result_box.clear()
        self._reset_result_overview()
        self._update_legend_strip()
        self._refresh_robot_preview()
        self.statusBar().showMessage(message)

    def _schedule_robot_preview_refresh(self) -> None:
        if hasattr(self, "_preview_timer"):
            self._preview_timer.start()

    def _refresh_robot_preview(self) -> None:
        if getattr(self, "current_results", None) is not None:
            return
        if not hasattr(self, "robot_canvas") or self.robot_canvas is None:
            return
        try:
            config = self._build_active_config()
            robot = self.viewmodel.robot_from_config(config)
            q_deg = self._parse_vector(self.q_init_edit.text())
            q_rad = np.deg2rad(q_deg)
            self.current_robot = robot
            self.robot_canvas.show_default_pose(robot, q_rad)
        except Exception as exc:
            _LOG.debug("Robot preview failed: %s", exc, exc_info=True)
            self.statusBar().showMessage(f"预览失败: {exc}")
            self.robot_canvas.refresh_display()

    def _update_legend_strip(self) -> None:
        if not hasattr(self, "legend_strip"):
            return
        if self.current_results is None:
            self.legend_strip.hide()
            return
        self.legend_strip.show()
        if not self._action_trajectories.isChecked():
            self.legend_strip.setText(TrajectoryPalette.trajectory_menu_hint_rich())
        else:
            self.legend_strip.setText(TrajectoryPalette.legend_tracks_rich())

    def _load_defaults(self) -> None:
        self._preview_timer.stop()
        self.animation_timer.stop()
        self.animation_result = None
        self.current_results = None
        if hasattr(self, "frame_slider"):
            self.frame_slider.blockSignals(True)
            self.frame_slider.setMaximum(0)
            self.frame_slider.setValue(0)
            self.frame_slider.blockSignals(False)

        rows = self.viewmodel.default_rows()
        self.parameter_table.blockSignals(True)
        for row_idx, row in enumerate(rows):
            values = {
                "joint": row["joint"],
                "name": row["name"],
                "a": row["a"],
                "alpha_deg": row["alpha_deg"],
                "d": row["d"],
                "offset_deg": row["offset_deg"],
                "qlim_min": row["qlim_deg"][0],
                "qlim_max": row["qlim_deg"][1],
            }
            for col_idx, (field_name, _) in enumerate(self.TABLE_COLUMNS):
                item = QTableWidgetItem(str(values[field_name]))
                self.parameter_table.setItem(row_idx, col_idx, item)
        self.parameter_table.blockSignals(False)

        default_state = self.viewmodel.default_state()
        simulation_defaults = self.viewmodel.default_simulation()

        for w in (self.q_init_edit, self.q_goal_edit, self.duration_edit, self.steps_edit):
            w.blockSignals(True)
        self.q_init_edit.setText(", ".join(map(str, default_state["q_init_deg"])))
        self.q_goal_edit.setText(", ".join(map(str, default_state["q_goal_deg"])))
        self.duration_edit.setText(str(simulation_defaults["duration_s"]))
        self.steps_edit.setText(str(simulation_defaults["steps"]))
        for w in (self.q_init_edit, self.q_goal_edit, self.duration_edit, self.steps_edit):
            w.blockSignals(False)
        self.result_box.clear()
        self._reset_result_overview()
        self._update_legend_strip()
        self._refresh_robot_preview()
        self.statusBar().showMessage("默认参数已恢复。")

    def _parse_vector(self, text: str) -> list[float]:
        values = [part.strip() for part in text.split(",") if part.strip()]
        return [float(value) for value in values]

    def _collect_link_rows(self) -> list[dict]:
        rows = []
        for row_idx in range(self.parameter_table.rowCount()):
            joint = self.parameter_table.item(row_idx, 0).text().strip()
            name = self.parameter_table.item(row_idx, 1).text().strip()
            a_val = float(self.parameter_table.item(row_idx, 2).text())
            alpha_val = float(self.parameter_table.item(row_idx, 3).text())
            d_val = float(self.parameter_table.item(row_idx, 4).text())
            offset_val = float(self.parameter_table.item(row_idx, 5).text())
            q0_val = float(self.parameter_table.item(row_idx, 6).text())
            q1_val = float(self.parameter_table.item(row_idx, 7).text())

            rows.append(
                {
                    "joint": joint,
                    "name": name,
                    "joint_type": "R",
                    "a": a_val,
                    "alpha_deg": alpha_val,
                    "d": d_val,
                    "offset_deg": offset_val,
                    "qlim_deg": [q0_val, q1_val],
                    "notes": "Edited in Qt UI.",
                }
            )
        return rows

    def _build_active_config(self) -> dict:
        return self.viewmodel.build_config(
            self._collect_link_rows(),
            self._parse_vector(self.q_init_edit.text()),
            self._parse_vector(self.q_goal_edit.text()),
            float(self.duration_edit.text()),
            int(float(self.steps_edit.text())),
        )

    def _run_simulation(self) -> None:
        try:
            config = self._build_active_config()
            export_root = Path.cwd() / "outputs"
            results = self.viewmodel.run_simulation(config, export_root=export_root)
            robot = self.viewmodel.robot_from_config(config)

            self.current_results = results
            self.current_robot = robot
            self._set_animation_result(results["cartesian_track"])
            self.robot_canvas.update_plot(robot, results["ptp"], results["cartesian_track"])
            self.joint_canvas.update_plot(results["ptp"])
            self._update_result_overview(results)
            self.result_box.setPlainText(self.viewmodel.summarize_results(results))
            self._update_legend_strip()
            self.statusBar().showMessage("仿真完成，结果已更新。")
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "仿真失败", str(exc))
            self.statusBar().showMessage("仿真失败，请检查参数。")

    def _run_trajectory_command(self) -> None:
        try:
            config = self._build_active_config()
            command = self.viewmodel.build_trajectory_command(
                self.command_edit.text(),
                self.trajectory_type_combo.currentText(),
                self.axis_combo.currentText(),
                float(self.radius_edit.text()),
                float(self.length_edit.text()),
                float(self.rise_edit.text()),
                float(self.turns_edit.text()),
                self._read_optional_circle_plane_z(),
            )
            results = self.viewmodel.run_trajectory_command(config, command)
            robot = self.viewmodel.robot_from_config(config)

            self.current_results = results
            self.current_robot = robot
            self._set_animation_result(results["cartesian_track"])
            self.robot_canvas.update_plot(robot, results["ptp"], results["cartesian_track"])
            self.joint_canvas.update_plot(results["cartesian_track"])
            self._update_result_overview(results)
            self.result_box.setPlainText(self.viewmodel.summarize_results(results))
            self._update_legend_strip()
            self.statusBar().showMessage(f"轨迹指令已模拟：{command.trajectory_type} / {command.axis}")
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "轨迹指令失败", str(exc))
            self.statusBar().showMessage("轨迹指令失败，请检查半径、长度和机械臂可达范围。")

    def _generate_task_blocks(self) -> None:
        try:
            task_blocks = self.viewmodel.parse_task_blocks(self.command_edit.text())
            if not task_blocks:
                QMessageBox.information(self, "未识别任务", "暂未从输入中识别到直线、圆或螺旋任务。")
                return
            self.current_task_blocks = task_blocks
            self._render_task_blocks(task_blocks)
            self._update_input_feasibility()
            self.statusBar().showMessage(f"已生成 {len(task_blocks)} 个任务块，请检查后执行。")
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "任务块生成失败", str(exc))

    def _schedule_feasibility_check(self) -> None:
        if not hasattr(self, "feasibility_timer"):
            return
        self.feasibility_timer.start()

    def _update_input_feasibility(self) -> None:
        if not hasattr(self, "feasibility_label"):
            return
        text = self.command_edit.text().strip()
        if not text:
            self.feasibility_label.setText("输入可行性：请输入自然语言任务后自动判断。")
            return

        try:
            task_blocks = self.viewmodel.parse_task_blocks(text)
            if not task_blocks:
                self.feasibility_label.setText("输入可行性：暂未识别到直线、圆或螺旋任务。")
                return
            config = self._build_active_config()
            precheck = self.viewmodel.precheck_task_blocks(config, task_blocks)
            self.feasibility_label.setText(self.viewmodel.summarize_precheck(precheck))
        except Exception as exc:  # noqa: BLE001
            self.feasibility_label.setText(f"输入可行性：暂时无法判断，请检查参数格式。原因：{exc}")

    def _render_task_blocks(self, task_blocks: list) -> None:
        self.task_table.setRowCount(len(task_blocks))
        for row_idx, task in enumerate(task_blocks):
            values = [
                task.task_type,
                task.axis,
                f"{task.radius_m:.4f}",
                f"{task.length_m:.4f}",
                f"{task.rise_m:.4f}",
                f"{task.turns:.2f}",
                "" if task.circle_plane_z_m is None else f"{task.circle_plane_z_m:.6f}",
                task.source_text,
            ]
            for col_idx, value in enumerate(values):
                self.task_table.setItem(row_idx, col_idx, QTableWidgetItem(str(value)))
        self.current_task_blocks = self._collect_task_blocks_from_table()

    def _collect_task_blocks_from_table(self) -> list[TaskBlock]:
        task_blocks = []
        for row_idx in range(self.task_table.rowCount()):
            task_type = self._task_cell_text(row_idx, 0)
            axis = self._task_cell_text(row_idx, 1)
            radius_m = float(self._task_cell_text(row_idx, 2))
            length_m = float(self._task_cell_text(row_idx, 3))
            rise_m = float(self._task_cell_text(row_idx, 4))
            turns = float(self._task_cell_text(row_idx, 5))
            cz_raw = self._task_cell_text(row_idx, 6).strip()
            source_text = self._task_cell_text(row_idx, 7)
            self._validate_task_block_fields(row_idx, task_type, axis, radius_m, length_m, rise_m, turns)
            circle_plane_z_m: float | None
            if cz_raw in {"", "-", "—"}:
                circle_plane_z_m = None
            else:
                circle_plane_z_m = float(cz_raw)
            task_blocks.append(
                TaskBlock(
                    task_type=task_type,
                    axis=axis,
                    radius_m=radius_m,
                    length_m=length_m,
                    rise_m=rise_m,
                    turns=turns,
                    circle_plane_z_m=circle_plane_z_m,
                    source_text=source_text,
                )
            )
        return task_blocks

    def _task_cell_text(self, row_idx: int, col_idx: int) -> str:
        item = self.task_table.item(row_idx, col_idx)
        return item.text().strip() if item else ""

    def _validate_task_block_fields(
        self,
        row_idx: int,
        task_type: str,
        axis: str,
        radius_m: float,
        length_m: float,
        rise_m: float,
        turns: float,
    ) -> None:
        row_label = row_idx + 1
        if task_type not in {"move_line", "draw_circle", "draw_helix"}:
            raise ValueError(f"第 {row_label} 个任务的类型无效，应为 move_line、draw_circle 或 draw_helix。")
        if axis not in {"x", "y", "z", "diagonal"}:
            raise ValueError(f"第 {row_label} 个任务的轴无效，应为 x、y、z 或 diagonal。")
        if radius_m < 0 or length_m < 0 or rise_m < 0:
            raise ValueError(f"第 {row_label} 个任务的半径、长度和抬升不能为负数。")
        if turns <= 0:
            raise ValueError(f"第 {row_label} 个任务的圈数必须大于 0。")

    def _move_selected_task_up(self) -> None:
        current_row = self.task_table.currentRow()
        if current_row <= 0:
            return
        self._swap_task_rows(current_row, current_row - 1)
        self.task_table.selectRow(current_row - 1)

    def _move_selected_task_down(self) -> None:
        current_row = self.task_table.currentRow()
        if current_row < 0 or current_row >= self.task_table.rowCount() - 1:
            return
        self._swap_task_rows(current_row, current_row + 1)
        self.task_table.selectRow(current_row + 1)

    def _swap_task_rows(self, first_row: int, second_row: int) -> None:
        for col_idx in range(self.task_table.columnCount()):
            first_text = self._task_cell_text(first_row, col_idx)
            second_text = self._task_cell_text(second_row, col_idx)
            self.task_table.setItem(first_row, col_idx, QTableWidgetItem(second_text))
            self.task_table.setItem(second_row, col_idx, QTableWidgetItem(first_text))
        self.current_task_blocks = self._collect_task_blocks_from_table()

    def _delete_selected_tasks(self) -> None:
        selected_rows = sorted({index.row() for index in self.task_table.selectedIndexes()}, reverse=True)
        if not selected_rows and self.task_table.currentRow() >= 0:
            selected_rows = [self.task_table.currentRow()]
        for row_idx in selected_rows:
            self.task_table.removeRow(row_idx)
        self.current_task_blocks = self._collect_task_blocks_from_table()
        self.statusBar().showMessage(f"已删除 {len(selected_rows)} 个任务块。")

    def _run_task_blocks(self) -> None:
        try:
            if not self.current_task_blocks:
                self._generate_task_blocks()
            if not self.current_task_blocks:
                return

            self.current_task_blocks = self._collect_task_blocks_from_table()
            if not self.current_task_blocks:
                QMessageBox.information(self, "暂无任务块", "请先输入自然语言指令并生成任务块。")
                return
            config = self._build_active_config()
            results = self.viewmodel.run_task_blocks(config, self.current_task_blocks)
            robot = self.viewmodel.robot_from_config(config)

            self.current_results = results
            self.current_robot = robot
            self._set_animation_result(results["cartesian_track"])
            self.robot_canvas.update_plot(robot, results["ptp"], results["cartesian_track"])
            self.joint_canvas.update_plot(results["cartesian_track"])
            self._update_result_overview(results)
            self.result_box.setPlainText(self.viewmodel.summarize_results(results))
            self._update_legend_strip()
            self.statusBar().showMessage(f"任务块执行完成：{len(self.current_task_blocks)} 个任务")
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "任务块执行失败", str(exc))
            self.statusBar().showMessage("任务块执行失败，请检查任务参数和可达范围。")

    def _set_animation_result(self, track_result: dict) -> None:
        self.animation_timer.stop()
        self.animation_result = track_result
        self.animation_frame = 0
        frame_count = len(track_result["q_rad"])
        self.frame_slider.blockSignals(True)
        self.frame_slider.setMaximum(max(frame_count - 1, 0))
        self.frame_slider.setValue(0)
        self.frame_slider.blockSignals(False)

    def _play_animation(self) -> None:
        if self.current_robot is None or self.animation_result is None:
            QMessageBox.information(self, "暂无运动", "请先运行一次仿真或轨迹指令。")
            return
        self.animation_timer.start()

    def _pause_animation(self) -> None:
        self.animation_timer.stop()

    def _reset_animation(self) -> None:
        self.animation_timer.stop()
        self.animation_frame = 0
        self.frame_slider.setValue(0)
        self._draw_animation_frame(0)

    def _advance_animation(self) -> None:
        if self.animation_result is None:
            self.animation_timer.stop()
            return
        max_frame = len(self.animation_result["q_rad"]) - 1
        if self.animation_frame >= max_frame:
            self.animation_timer.stop()
            return
        self.animation_frame = min(self.animation_frame + 6, max_frame)
        self.frame_slider.setValue(self.animation_frame)

    def _seek_animation(self, frame_idx: int) -> None:
        self.animation_frame = int(frame_idx)
        self._draw_animation_frame(self.animation_frame)

    def _draw_animation_frame(self, frame_idx: int) -> None:
        if self.current_robot is None or self.animation_result is None:
            return
        self.robot_canvas.update_frame(self.current_robot, self.animation_result, frame_idx)

    def _reset_result_overview(self) -> None:
        if not hasattr(self, "status_pill"):
            return
        self.status_pill.setText("未运行")
        self.status_pill.setStyleSheet("background: #f2f2f7; color: #6e6e73;")
        self.status_message_label.setText("在「设置 → 仿真设置」中运行仿真，这里会显示是否成功。")
        self.max_error_value.setText("-- m")
        self.mean_error_value.setText("-- m")
        self.ik_failure_value.setText("--")
        self.final_position_value.setText("--")
        if hasattr(self, "legend_strip"):
            self.legend_strip.hide()

    def _update_result_overview(self, results: dict) -> None:
        assessment = self.viewmodel.assess_results(results)
        self.status_pill.setText(assessment["status_text"])
        if assessment["level"] == "success":
            self.status_pill.setStyleSheet("background: #e8f7ee; color: #16833a;")
        elif assessment["level"] == "warning":
            self.status_pill.setStyleSheet("background: #fff4df; color: #a45a00;")
        else:
            self.status_pill.setStyleSheet("background: #ffecec; color: #b42318;")
        self.status_message_label.setText(assessment["message"])
        self.max_error_value.setText(f"{assessment['max_error_m']:.4f} m")
        self.mean_error_value.setText(f"{assessment['mean_error_m']:.4f} m")
        self.ik_failure_value.setText(f"{100 * assessment['ik_failure_ratio']:.1f}%")
        final_pos = assessment["final_position_m"]
        self.final_position_value.setText(f"[{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]")

    def _export_results(self) -> None:
        if not self.current_results:
            QMessageBox.information(self, "暂无结果", "请先运行一次仿真。")
            return

        target_dir = QFileDialog.getExistingDirectory(self, "选择导出目录")
        if not target_dir:
            return

        try:
            self.viewmodel.export_current_results(self.current_results, target_dir)
            self.statusBar().showMessage(f"结果已导出到: {target_dir}")
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "导出失败", str(exc))

    def _export_robot_program(self) -> None:
        if not self.current_results:
            QMessageBox.information(self, "暂无结果", "请先运行一次轨迹任务。")
            return

        target_dir = QFileDialog.getExistingDirectory(self, "选择机器人程序导出目录")
        if not target_dir:
            return

        try:
            output_path = self.viewmodel.export_robot_program(self.current_results, target_dir)
            QMessageBox.information(self, "导出成功", f"机器人程序已导出:\n{output_path}")
            self.statusBar().showMessage(f"机器人程序已导出: {output_path}")
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "机器人程序导出失败", str(exc))

    def _dry_run_send_program(self) -> None:
        if not self.current_results:
            QMessageBox.information(self, "暂无结果", "请先运行一次轨迹任务。")
            return

        try:
            message = self.viewmodel.dry_run_send_program(self.current_results)
            QMessageBox.information(self, "虚拟下发完成", message)
            self.statusBar().showMessage(message)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "虚拟下发失败", str(exc))
