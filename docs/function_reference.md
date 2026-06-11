# 六自由度机械臂 Studio — 函数说明全书

本文档按**源码文件**列出本仓库主干代码中的**类、方法与模块级函数**，并说明其作用与典型调用关系（与 `release/robot_app_test_package` 重复的打包副本不另列，以根目录 `python/`、`qt_app/` 为准）。

**约定**

- 名称以单下划线 `_` 开头的方法/函数视为**模块或类内部实现细节**，一般仅被同文件或紧邻层调用。
- 类型标注中的 `dict` 结构与轨迹字段细节可对照 `docs/software_architecture_visual.md` 第 2 节。

---

## `python/robot_model.py` — 机器人模型与运动学核心

### 模块级函数

| 函数 | 作用 |
|------|------|
| `workspace_root()` | 返回包根目录上一级（项目根）的 `Path`，用于定位 `config/` 等资源。 |
| `default_config_path()` | 返回默认 JSON 配置路径 `config/robot_mdh_template.json`。 |
| `load_robot_config(config_source)` | 从 `None`（默认路径）、文件路径或已有 `dict` 加载并**深拷贝**得到机器人配置字典（MDH 连杆、单位、环境、仿真默认等）。 |
| `transform_from_rpy_translation(rpy_deg, translation_m)` | 由绕 x-y-z 的欧拉角（度）与平移（米）拼装 4×4 齐次变换矩阵。 |
| `mdh_transform(a, alpha_rad, d, theta_rad)` | 单节**改进 D-H**（Modified DH）连杆的 4×4 变换矩阵。 |
| `standard_dh_transform(a, alpha_rad, d, theta_rad)` | 单节**标准 D-H**矩阵；`SixDofRobot` 在 `convention` 为 `standard_dh` 时使用。 |
| `homogeneous_rz` / `homogeneous_tx` / `homogeneous_rx` | 绕 z/x 的旋转、沿 x 平移的齐次矩阵；用于复合底座等局部变换。 |
| `rotation_error(current_r, target_r)` | 基于两旋转矩阵列向量叉积近似的**姿态误差向量**（用于全位姿 IK）。 |
| `rpy_deg_from_transform(transform)` | 从旋转子矩阵提取 roll–pitch–yaw（度），供位姿展示与 CSV 导出。 |
| `_length_scale_from_config(config)` | 根据 `config["units"]["length"]` 判断 JSON 中长度字段是否按毫米解释，返回 1.0 或 1e-3。 |
| `_translation_vector_m(section, scale)` | 将 `base_transform` / `tool_transform` 中的平移统一转为**米**（支持 `translation_mm`）。 |
| `_endpoint_offset_in_frame6_m(config)` | 解析 `endpoint7` 在连杆 6 坐标系下的偏移（mm 或按单位缩放的 m）。 |
| `sampled_segment_segment_distance_m(a0,a1,b0,b1,samples)` | 两线段间最近距离的**网格采样近似**，用于非相邻连杆段的净空估计。 |

### 类 `SixDofRobot`

| 方法 | 作用 |
|------|------|
| `__init__(config)` | 读入配置，建立 `base`、`tool`、各连杆 `a/d/alpha/offset/qlim`，处理 `compound_base` 与 `environment`；后续所有 FK/IK 基于此实例。 |
| `ground_plane_z_m()` | 读取地平面世界 z（支持 `ground_plane_z_mm`）。 |
| `joint_z_constraint_floor_m()` | 关节允许的最低高度 = 地平面 z + 配置中的地面净空（mm）。 |
| `joint_indices_above_ground_0based()` | 需要做「高于地面」检查的关节索引（由配置 `check_joints_1based_above_ground` 得到 0-based）。 |
| `ground_penalty(q_rad)` | 被选关节原点低于阈值高度时的**平方惩罚标量**，供位姿 IK 软约束。 |
| `ground_penalty_grad(q_rad, eps)` | 上述惩罚对 `q` 的**数值梯度**。 |
| `min_checked_joint_height_m(q_rad)` | 被检关节转轴原点 z 的最小值。 |
| `min_checked_joint_clearance_above_floor_m(q_rad)` | 最小高度与地板阈值的差，≤0 表示违反约束。 |
| `link_envelope_radius_m()` | 连杆包络圆柱半径（米），从 `link_envelope_radius_mm` 换算。 |
| `min_non_adjacent_segment_clearance_m(q_rad)` | 非相邻连杆段最小距离减去两倍包络半径，用于自碰/间隙度量。 |
| `min_non_adjacent_segment_separation_raw_m(q_rad)` | 同上但不减包络，为原始折线段间距。 |
| `clamp_to_limits(q_rad)` | 将关节角裁剪到 `qlim` 内。 |
| `fkine(q_rad)` | **正运动学**：返回末端（含 tool）4×4 位姿。 |
| `_forward_accumulate(q_rad)` | 内部：逐节累乘 DH，得到连杆坐标系列表及**未乘 tool** 的累积变换。 |
| `forward_chain(q_rad)` | 返回从基座到末端（含 TCP）的齐次矩阵列表。 |
| `joint_positions` / `joint_positions_display(q_rad)` | 折线骨架用的 3D 关节点序列（显示与碰撞用一致）。 |
| `joint_rotation_axes_world(q_rad)` | 各关节轴在世界系下的**原点**与**单位方向**（复合底座下前两轴方向特殊处理）。 |
| `joint_markers_with_tcp(q_rad)` | 六个关节参考点 + TCP 位置，供可视化球体/标签。 |
| `pose_error(q_rad, target_transform)` | 6 维误差（位置 + 姿态），用于全位姿 IK。 |
| `numerical_jacobian(q_rad, eps)` | 6×n **数值雅可比**（位置对 q 的差分 + 姿态列差分）。 |
| `inverse_kinematics(target_transform, q0_rad, ...)` | **全位姿阻尼最小二乘 IK**，迭代至误差范数小于容差或残差可接受。 |
| `inverse_kinematics_position_only(target_position_m, q0_rad, ...)` | **位置-only IK**（姿态不约束），用 3×n 雅可比分块；轨迹跟踪与默认笛卡尔圆常用。 |
| `pose_dict(q_rad)` | 汇总为含 `q_deg`、`position_m`、`rpy_deg`、`transform` 的字典，便于 JSON/界面展示。 |

---

## `python/trajectory.py` — 标称轨迹与评测

| 函数 | 作用 |
|------|------|
| `time_vector(duration_s, steps)` | 在 `[0, duration_s]` 上均匀生成 `steps` 个时刻。 |
| `quintic_blend(tau)` | 归一化时间 τ∈[0,1] 上的**五次多项式**平滑插值因子 s(τ)。 |
| `joint_ptp_trajectory(q_start, q_goal, duration_s, steps)` | **关节空间 PTP**：用五次混合在起始/目标关节角之间插补，返回 `(t, q_traj)`。 |
| `plane_unit_vectors(plane, center_hint_m)` | 对 `yz`/`xz`/`xy`（默认）平面给出圆轨迹的径向与切向单位矢量。 |
| `cartesian_circle_positions(start, radius_m, center_hint, plane, steps)` | 生成过起点、给定平面与半径的**闭合圆采样点**（首尾强制过起点）。 |
| `poses_from_positions(positions_m, reference_transform)` | 保留参考帧姿态、只平移末端位置，得到位姿矩阵列表（旧接口辅助）。 |
| `reachable_reference_trajectory(robot, q_start_rad, duration_s, steps)` | 当 IK 失败时的**备选**：关节空间正弦扰动并限幅，仍返回时间及对应 FK 末端轨迹。 |
| `cartesian_track_trajectory(robot, q_start_rad, duration_s, steps)` | **默认仿真用**笛卡尔圆：从配置读半径/平面/中心偏置，逐步 `inverse_kinematics_position_only`；失败则退回 `reachable_reference_trajectory`。 |
| `evaluate_trajectory(robot, time_s, q_traj, desired_positions_m, source, trajectory_type)` | 对一整条关节轨迹做 FK，计算 `position_m`、`desired_position_m`、误差 **`metrics`**、`transforms`、`rpy_deg` 等标准结果字典。 |

---

## `python/simulator.py` — 默认仿真管线与导出

| 函数 | 作用 |
|------|------|
| `_to_serializable(value)` | 递归将 `np.ndarray`、numpy 标量等转为 JSON 可写类型。 |
| `export_results(output_dir, result)` | 将单条 `evaluate_trajectory` 结果写出 `joint_trajectory.csv`、`pose_trajectory.csv`、`simulation_result.json`。 |
| `run_fk_demo(config_source)` | 仅根据 `q_init`/`q_goal` 做 FK，返回 `robot_name`、初末 `pose_dict`、`links` 等轻量 JSON 友好结构。 |
| `run_default_simulation(config_source, output_root)` | **默认双通道仿真**：PTP 关节轨迹 + 配置驱动的笛卡尔圆跟踪，各自 `evaluate_trajectory`；合并为 `fk_demo`（再调 `run_fk_demo`）、`ptp`、`cartesian_track`；若给 `output_root` 则导出子目录。 |

---

## `python/services/simulation_service.py`

| 方法 | 作用 |
|------|------|
| `run_default(config, output_root)` | 薄封装 `run_default_simulation`，供 ViewModel 调用。 |
| `run_fk(config)` | 薄封装 `run_fk_demo`。 |
| `export_results_bundle(results, export_root)` | 将成套 `results` 中的 `ptp` 与 `cartesian_track` 分别 `export_results` 到子目录。 |
| `robot_from_config(config)` | 返回 `SixDofRobot(config)`。 |

---

## `python/services/kinematics_service.py`

| 类型/方法 | 作用 |
|-----------|------|
| `IKRequest` | 数据类：目标位置、可选目标 RPY、可选种子 `q_seed_deg`。 |
| `IKResult` | 数据类：是否成功、`q_solution_deg`、残余误差信息等。 |
| `KinematicsService.__init__(config)` | 持有配置并构造 `SixDofRobot`。 |
| `solve_inverse(request)` | 由目标位姿调用 `inverse_kinematics` 或退化为位置目标，封装为 `IKResult`。 |
| `_seed_rad` | 将种子关节角转为弧度向量。 |
| `_target_transform` | 位置 + RPY → 4×4 目标矩阵。 |
| `_orientation_error_deg` | 当前姿态与用户指定 RPY 的误差角（可选）。 |

---

## `python/services/language_service.py`

| 类型/方法 | 作用 |
|-----------|------|
| `TaskBlock` | 描述一条结构化子任务：`move_line` / `draw_circle` / `draw_helix` 及几何参数与原文。 |
| `TaskBlock.to_command()` | 映射为 `TrajectoryCommand`（`line` / `circle` / `spatial_curve`）。 |
| `LanguageTaskService.parse(text)` | 主入口：**规则 + 正则**的中文/简单英文离线解析，得到 `TaskBlock` 列表。 |
| `_split_clauses` | 按「然后」「接着」、中英文逗号/分号/换行切分子句。 |
| `_parse_clause` | 单句识别螺旋/圆/直线，抽出轴、半径、长度、抬升、圈数、水平圆高度等。 |
| `_parse_axis` | 从「沿 x 轴」「xy 平面」等字面量映射到轨迹服务使用的轴符号。 |
| `_parse_distance` / `_parse_number` | 在子句中提取数值并处理 mm/cm/m 单位。 |
| `_parse_optional_circle_plane_z` | **仅圆轨迹**：解析世界 z（如「平面 z=50mm」），与螺旋 rise 区分开。 |
| `parse_circle_plane_z(text)` | 对外单列 API：仅从文本中取水平圆 z（米）；未写明返回 `None`。 |

---

## `python/services/trajectory_service.py`

### `TrajectoryCommand`（数据类）

冻结数据类：`trajectory_type`（`line`/`circle`/`spatial_curve`）、`axis`、`radius_m`、`length_m`、`rise_m`、`turns`、`circle_plane_z_m`（水平圆可选世界 z）。

### `TrajectoryService` — 公共方法

| 方法 | 作用 |
|------|------|
| `run_command(config, command)` | 单命令：`run_commands` 的单元素包装。 |
| `run_commands(config, commands)` | **核心**：用 `SixDofRobot(config)`，自动选起点、`precheck_commands`，对每段 `_best_track` 生成 `desired_positions`/`q_track`，拼接后调用 `evaluate_trajectory`，并写入 `metrics`（IK 失败、雅可比条件数、关节步长与环境间隙等）、`precheck`、`commands`、`segment_diagnostics` 等。**返回即 `cartesian_track` 同源字典**。 |
| `precheck_config_commands(config, commands)` | 不执行完整跟踪时的工作空间/`precheck` 报告（可行性、缩放建议、`auto_start` 等），供界面「可行性」标签。 |

### `TrajectoryService` — 私有方法（概要）

这些方法共同实现「工作空间采样 → 风险 → 缩放/起点 → IK 跟踪」流水线；除调试外无需自外部调用。

| 方法簇 | 作用 |
|--------|------|
| `_apply_ik_preview_to_precheck` | 将自动起点的 IK 试跑摘要合并进 `precheck` 标志（如是否通过预览）。 |
| `_select_auto_start_pose` | 在给定默认 `q_init` 与轨迹命令集合下搜索较优起始关节角（可含 IK 预览）。 |
| `_score_start_pose_tracking` / `_auto_start_candidates` | 对候选起始姿态打分 / 枚举候选集合。 |
| `precheck_commands` | **预检**：工作空间地图、各段缩放建议、`risk_level`、`segments`、`auto_start` 等骨架。 |
| `_estimated_workspace_radius` / `_workspace_map` / `_workspace_map_key` | 在当前机器人上构建或缓存**工作空间点云**（球形范围内 FK 采样），用于点到云距离。 |
| `_nearest_workspace_map_distances` / `_workspace_map_scale` | 目标离散点相较工作空间的距离与建议整体缩放系数。 |
| `_command_extent` / `_recommended_scale` / `_recommend_center_shift` / `_recommend_initial_posture` | 由命令几何与可达性估计建议尺度/中心/姿态提示。 |
| `_precheck_risk_level` | 综合距离、越界比例等输出 `low`/`medium`/`high`。 |
| `_environment_motion_metrics` | 沿整条 `q_track` 统计关节高度、地面净空、非邻段间距等标量，写入 `metrics`。 |
| `_segment_diagnostics` | 单段的误差、失败步数等诊断 dict。 |
| `_joint_motion_metrics` | 相邻帧关节步长的 max/mean/RMS（度）。 |
| `_condition_numbers` | 位置雅可比 3×n 的条件数序列，用于奇异性粗检。 |
| `_best_track` | 对单条 `TrajectoryCommand` 在多种**尺度候选**下尝试生成 `desired_positions` 与逐步 DLS IK，选优返回。 |
| `_candidate_scales` | 生成用于重试的缩放列表（含 1.0 与预检建议）。 |
| `_desired_positions` / `_line_positions` / `_circle_positions` / `_circle_start_radial` / `_spatial_curve_positions` / `_track_positions` | 按命令类型生成世界系末端**目标位置序列**（含平面圆、螺旋、斜向等）。 |
| `_solve_position_step_dls` | 单步**阻尼最小二乘**位置 IK，带地面惩罚与奇异性处理，供轨迹跟踪内层循环。 |
| `_refine_q_ground_and_position` | 在跟踪后对单步姿态做接地/位置的细化修正。 |
| `_joint_limit_centering_target` | 关节限位中心吸引项，减小贴边求解风险。 |
| `_axis_vector` / `_orthogonal_basis` | 世界系轴向量与构造圆/弧局部标架。 |

---

## `python/services/controller_service.py`

| 类型/方法 | 作用 |
|-----------|------|
| `ControllerReport` | 下发程序后的摘要：控制器名、是否接受、文案、可选路径与采样点数。 |
| `RobotController`（Protocol） | 未来真实硬件控制器需实现的接口（当前仅 DryRun）。 |
| `DryRunController.send_program` | 不真实下发，返回接受状态与点数统计（教学演示）。 |
| `RobotProgramService.build_program(results)` | 从仿真 `results` 抽取时间与关节/笛卡尔采样，打成 `robot_task_program_v1` JSON 字典（含 diagnostics、task_blocks/commands 镜像）。 |
| `export_program` | 写入 `robot_program_dry_run.json`。 |
| `dry_run_send` | `build_program` + `DryRunController.send_program`。 |

---

## `python/cli.py`

| 函数 | 作用 |
|------|------|
| `main()` | 命令行入口：通常调用默认仿真并可写盘（与 README/用法一致时使用）。 |

---

## `python/compare_results.py`

| 函数 | 作用 |
|------|------|
| `load_pose_csv(path)` | 读入轨迹 CSV（若项目用于对比 MATLAB 等输出的位姿序列）。 |
| `main()` | 对比两套结果文件的脚本入口。 |

---

## `qt_app/main.py`

| 函数 | 作用 |
|------|------|
| `main()` | Qt 程序入口：配置 Matplotlib、`apply_app_style`、装载 `SimulationViewModel`、创建 `MainWindow`、运行事件循环。 |

---

## `qt_app/styles.py`

| 函数 | 作用 |
|------|------|
| `apply_app_style(app)` | 对整个 `QApplication` 注入统一 QSS（深色顶栏、卡片、表格、滑动条等），保证 Studio 观感一致。 |

---

## `qt_app/viewmodels/simulation_viewmodel.py` — ViewModel

| 方法 | 作用 |
|------|------|
| `__init__(config_path)` | 载入 JSON 模板，构造 `SimulationService`、`TrajectoryService`、`LanguageTaskService`、`RobotProgramService`。 |
| `config`（property） | 返回当前模板配置的**深拷贝**（防止界面直接改内部引用）。 |
| `default_rows` / `default_state` / `default_simulation` | 分别是 `links`、`default_state`、`simulation_defaults` 的深拷贝，供界面初始化。 |
| `build_config(...)` | 用表格关节行与用户编辑的仿真参数拼装**完整运行时 `config`**。 |
| `validate_config(config)` | 校验连杆数量、关节限序、向量长度、`steps`、`duration_s` 等，不通过抛出 `ValueError`。 |
| `run_simulation` | `validate` 后调用 `SimulationService.run_default`，并附加 `active_config`。 |
| `run_trajectory_command` | `validate` + 单命令轨迹 + `_wrap_track_result`。 |
| `run_task_blocks` | 多块任务：`task.to_command()` 串联 `run_commands`，并把 `task_blocks` 快照写入轨迹结果，再 wrap。 |
| `precheck_task_blocks` | 仅可行性分析，不写仿真结果字典。 |
| `summarize_precheck(precheck)` | **人类可读**一行/多行可行性摘要（风险等级、建议缩小半径等）。 |
| `_wrap_track_result(config, track_result)` | 组装与其它入口一致的 **`results` 四套键**：`run_fk`→`fk_demo`，`track_result` 深拷贝改写 `summary` 得 `ptp`，原轨为 `cartesian_track`，外加 `active_config`。 |
| `parse_task_blocks` | 委派 `LanguageTaskService.parse`。 |
| `build_trajectory_command(...)` | 由 UI 参数构造 `TrajectoryCommand`，若有自由文本再 `_parse_trajectory_text`。 |
| `_parse_trajectory_text` | 解析命令行面板中的中英文几何覆盖（半径、长度等关键词）。 |
| `_extract_number` | 从文段提取浮点参数的辅助函数。 |
| `summarize_results(results)` | 根据 `cartesian_track` 的摘要、metrics、预检信息拼**多段落文本**，供右侧结果框。 |
| `_precheck_lines` / `_diagnostic_lines` / `_segment_diagnostic_lines` | 将 `precheck`、`metrics`、`segment_diagnostics` 转为列表文本行。 |
| `_was_auto_scaled` | 判断是否发生过自动缩放（用于摘要话术）。 |
| `_task_summary_lines` / `_command_summary_lines` | 多任务块与单指令的简述行。 |
| `assess_results(results)` | 对误差、IK 失败率、条件数等做**分级评估** dict，供 UI 或后续逻辑。 |
| `export_current_results` | 调 `SimulationService.export_results_bundle`。 |
| `export_robot_program` / `dry_run_send_program` | 委派 `RobotProgramService` 导出或虚拟下发并返回可读消息/`ControllerReport`。 |
| `robot_from_config` | 返回画布用 `SixDofRobot`。 |

---

## `qt_app/views/main_window.py` — 主视图 `MainWindow`

下列按**职责分组**列出方法（括号内 `_` 开头为私有槽/辅助）。

### 构造与外壳

| 方法 | 作用 |
|------|------|
| `__init__(viewmodel)` | 保存 ViewModel，初始化仿真/动画/防抖定时器为空状态，`_build_ui`、菜单信号。 |
| `_build_ui` | 拼装顶栏、左栏（参数 / 仿真 / 任务 / 导引）、中部 3D、底部关节图与播放条、右侧概览与结果文本等。 |

### 菜单 / 偏好 / 顶栏

| 方法 | 作用 |
|------|------|
| `_build_menu_bar` | 调试轨迹叠加、轨迹显示、`设置`对话框动作。 |
| `_on_debug_toggled` / `_on_trajectories_toggled` | 切换 `RobotCanvas` 的调试层与轨迹线显示。 |
| `_redraw_robot_canvas` | 在无新仿真时按当前 robot/results 刷新 3D。 |
| `_open_settings` / `_ensure_settings_dialog` | 弹出或惰性创建设置对话框（仿真参数宿主）。 |
| `_build_top_bar` | Studio 标题行与顶层控件容器。 |

### 轨迹与自然语言卡片

| 方法 | 作用 |
|------|------|
| `_build_trajectory_command_card` | 单行命令输入、运行按钮、可行性标签、展开的轨迹类型面板、任务生成/执行任务块表格等 UI。 |
| `_toggle_trajectory_tools` | 展开/收起高级轨迹参数面板。 |
| `_run_trajectory_input` | 运行按钮逻辑：若有文本且解析出 `task_blocks`，则 `_render_task_blocks` 后立即 `_run_task_blocks`；否则走 `_run_trajectory_command()`（仅用轨迹面板参数）。 |
| `_read_optional_circle_plane_z` | 从控件读水平圆 z，空则为 `None`。 |
| `_on_trajectory_command_type_changed` | 下拉切换线/圆/螺旋时切换可见参数。 |
| `_on_trajectory_axis_for_circle_plane` | 圆平面轴与 z 编辑器联动辅助。 |

### 参数与仿真卡片

| 方法 | 作用 |
|------|------|
| `_build_parameter_group` | 六级 MDH/`qlim`/关节名表格。 |
| `_build_simulation_group` | `q_init`/`q_goal`、时长、步数、运行仿真、重置、清空预览、导出仿真等。 |
| `_add_compact_param` | 格子布局中加一行「标签 + 控件」的助手。 |

### 中间 3D、动画条、图表与结果文本

| 方法 | 作用 |
|------|------|
| `_build_animation_strip` | 播放 / 暂停 / 复位 / 快进 / 导出程序 / DryRun 导出、帧滑条、图例 strip。 |
| `_build_joint_plot_card` | 容纳 `JointPlotCanvas`。 |
| `_build_result_text_card` | 容纳 `result_box`（`QPlainTextEdit`）。 |
| `_build_guidance_group` | 导引说明卡片。 |

### 结果概览.metric 卡片与参数变更

| 方法 | 作用 |
|------|------|
| `_build_result_overview_card` | 误差、IK 失败率、末端位置等仪表卡片网格。 |
| `_create_metric_card` | 创建单个标题 + 大号数值 QLabel 的卡面。 |
| `_on_parameter_table_changed` | DH 表格改动后防抖刷新预览并使结果失效。 |
| `_clear_results_and_preview` / `_invalidate_simulation_results` | 清空 `current_results`、动画并重置文案/仪表。 |

### Robot 预览（未跑完整仿真时）

| 方法 | 作用 |
|------|------|
| `_schedule_robot_preview_refresh` | 防抖定时触发 `_refresh_robot_preview`。 |
| `_refresh_robot_preview` | 用当前表格构建 config → `robot_from_config` → 默认姿态画布预览。 |

### 图例与其它 UI 辅助

| 方法 | 作用 |
|------|------|
| `_update_legend_strip` | 根据调色板/`TrajectoryPalette` 更新轨迹图例字符串。 |

### 默认与配置收集

| 方法 | 作用 |
|------|------|
| `_load_defaults` | 从 ViewModel 读出默认表格行和仿真默认填表。 |
| `_parse_vector` | 解析逗号分隔的角度/数值向量。 |
| `_collect_link_rows` | 从 `parameter_table` 收集 6 行 dict。 |
| `_build_active_config` | **核心**：`_collect_link_rows` + 解析编辑框 → `viewmodel.build_config`。 |

### 仿真与轨迹执行

| 方法 | 作用 |
|------|------|
| `_run_simulation` | 默认 PTP+笛卡尔圆管线，写 `current_results`、更新 robot、画布、关节图、overview、摘要。 |
| `_run_trajectory_command` | 读取轨迹面板参数 → `build_trajectory_command` → `run_trajectory_command`。 |
| `_generate_task_blocks` | `command_edit` → `parse` → `_render_task_blocks`。 |
| `_schedule_feasibility_check` | 防抖后 `_update_input_feasibility`。 |
| `_update_input_feasibility` | `precheck_task_blocks` + `summarize_precheck` → `feasibility_label`。 |
| `_render_task_blocks` | 表格行与 `current_task_blocks` 同步展示。 |

### 任务表编辑与执行

| 方法 | 作用 |
|------|------|
| `_collect_task_blocks_from_table` | 表格 → `TaskBlock` 列表；带字段校验。 |
| `_task_cell_text` | 读取单元格文本。 |
| `_validate_task_block_fields` | 行级合法性校验。 |
| `_move_selected_task_up` / `_move_selected_task_down` / `_swap_task_rows` / `_delete_selected_tasks` | 多段任务排序与删除。 |
| `_run_task_blocks` | 收集表格任务 → `run_task_blocks`。 |

### 动画驱动

| 方法 | 作用 |
|------|------|
| `_set_animation_result` | 绑定用于播放的时间序列字典（常为 `cartesian_track`）、重置帧。 |
| `_play_animation` / `_pause_animation` / `_reset_animation` | 定时器播放控制。 |
| `_advance_animation` | 时钟槽：快进若干帧循环。 |
| `_seek_animation` / `_draw_animation_frame` | 滑条与用户 scrub；调 `RobotCanvas.update_frame`。 |

### 结果展示与导出

| 方法 | 作用 |
|------|------|
| `_reset_result_overview` / `_update_result_overview` | 仪表卡片占位或从 `metrics`/`position_m` 尾点刷新。 |
| `_export_results` | 调 ViewModel 导出 CSV/JSON 包。 |
| `_export_robot_program` | 写 `robot_program_dry_run.json`。 |
| `_dry_run_send_program` | 虚拟控制器接收提示（消息框或状态）。 |

### 样式小工具

| 方法 | 作用 |
|------|------|
| `_apply_left_panel_shadow` | 左栏 `QFrame` 阴影效果。 |

---

## `qt_app/widgets/robot_canvas.py` — `RobotCanvas`

| 方法 | 作用 |
|------|------|
| `__init__` | 建立 Matplotlib 3D 轴、占位图、UR5e 网格绑定等。 |
| `set_display_options` | 切换调试叠加、轨迹折线显示。 |
| `refresh_display` | 重绘当前 Figure。 |
| `_draw_placeholder` | 无数据时的空场景。 |
| `show_default_pose` | 单次 FK 骨架 + 可选 STL。 |
| `update_plot` | **主绘图**：TCP 轨迹、期望轨迹、关节扫掠、实心/骨架模式，自动相机范围。 |
| `update_frame` | 播放器按帧只画当前姿态（可能含运动模糊扫掠）。 |
| `_camera_center_radius` / `_apply_camera_limits` / `_fit_camera` | 根据点云拟合视图中心与缩放。 |
| `_set_axis_labels_visible` | 显隐坐标轴刻度标签。 |
| `_draw_motion_sweep` | 关节空间插值半透明「扫掠」效果。 |
| `_draw_visual_or_solid_robot` | 若有 UR5e 网格则用 STL，否则圆柱骨架。 |
| `_display_geometry` | 连杆骨架折线绘制。 |
| `_draw_solid_robot` | STL 网格与坐标轴辅助。 |
| `_draw_cylinder_between` / `_draw_sphere` | 骨架几何绘制。 |
| `_orthogonal_basis` | 给定方向构造局部坐标系。 |
| `_draw_joint_rotation_axes_from` | 可选绘制关节轴箭头。 |
| `_draw_joint_labels` / `_draw_start_end_points` / `_draw_world_axes` | 标注与世界系三轴。 |
| `_style_axes` | 统一轴背景、网格、宽高比等。 |

---

## `qt_app/widgets/joint_plot_canvas.py` — `JointPlotCanvas`

| 方法 | 作用 |
|------|------|
| `__init__` | 建 Figure 与子图网格。 |
| `_draw_placeholder` | 无量测时的提示。 |
| `update_plot(result)` | 从 `evaluate_trajectory` 风格字典读 `time_s`、`q_deg`（等）绘制每条关节曲线。 |

---

## `qt_app/widgets/plot_palette.py` — `TrajectoryPalette`

类属性：`desired`、`actual`、`ptp`、`tcp`、`start`、`end`、`hint_gray` 等末端/轨迹色谱。下列 **类方法**返回 Qt 可用的富文本/HTML 小段：

| 方法 | 作用 |
|------|------|
| `legend_tracks_rich` | 侧边图例：期望、实际、PTP、起点、终点等着色圆点。 |
| `trajectory_menu_hint_rich` | 灰色小字，提示在菜单勾选「显示轨迹」。 |

---

## `qt_app/widgets/plot_style.py`

| 函数 | 作用 |
|------|------|
| `configure_matplotlib_fonts()` | 在 Windows/macOS/Linux 选择可用的中文/unicode 后备字体并设置 matplotlib `rcParams`，避免负号/CJK 缺字。 |

---

## `qt_app/widgets/ur5e_visual_model.py` — STL 可视化

### `MeshBinding`（`@dataclass`）

冻结数据类：`name`、`filename`、`frame_index`、`color`、`alpha`、`local_xyz`、`local_rpy`，描述某连杆 STL 相对于 FK 连杆系的显示绑定。常量 **`UR5E_COLLISION_BINDINGS`**：预置 UR5e 七段碰撞网格文件名与姿态偏置，`UR5eVisualModel.draw` 依此挂载网格。

### `UR5eVisualModel`

| 方法 | 作用 |
|------|------|
| `__init__(mesh_dir)` | 指定碰撞/视觉网格目录。 |
| `is_available` / `supports` / `missing_files` | 判断是否可画指定 `SixDofRobot`、缺失哪些文件名。 |
| `draw` | 在 mpl `axes` 上绘制六位连杆三角网格（若文件齐）。 |
| `link_polyline` | 对齐 `SixDofRobot.joint_positions` 显示级骨架折线顶点。 |
| `joint_markers_with_tcp` / `joint_rotation_axes_world` | 与机器人模型语义一致的关节点与轴（供画布调用）。 |
| `_assembled_link_frames` | UR5e 标准 DH 下各连杆在世界系的位姿序列。 |
| `_load_mesh` / `_read_binary_stl` / `_read_ascii_stl` | STL 二进制/ ASCII 极简解析读取三角面顶点。 |
| `_transform_triangles` / `_lit_facecolors` | 顶点乘齐次矩阵、简单光照设色。 |
| `_ur5e_link_frames` / `_tf` / `_rz` | UR5e 固定几何常数推导（与本项目 FK 并联用于网格对齐）。 |

---

## `scripts/download_ur5e_meshes.py`

| 函数 | 作用 |
|------|------|
| `main()` | 从网络拉取 UR5e 网格资源到本地 `assets`（需在合法网络环境与许可下使用）。 |

---

## `scripts/validate_workspace_trajectories.py`

| 函数 | 作用 |
|------|------|
| `frange` | 简易浮点步进范围（脚本内网格扫描）。 |
| `_metrics_from_track` | 从轨迹字典提取 IK 失败率、最大误差等指标行。 |
| `run_fixed_mode` | **固定网格**枚举线长/半径组合跑批，汇总可达性统计。 |
| `run_prod_mode` | **生产式**更广参数扫描版本（与课上验收脚本意图一致时使用）。 |
| `summarize` | 控制台打印聚合表。 |
| `main()` | argparse 入口，选模式与步数、范围。 |

---

## 维护提示

新增公共 API 时请同步更新本节；若重构 `TrajectoryService` 或 `MainWindow`，优先核对本节中「**核心**」「**数据源**」「**exported keys**」描述是否仍然成立。
