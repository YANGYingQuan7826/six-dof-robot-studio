# 六自由度机械臂仿真软件 · 架构与数据流（可视化）

与 **`docs/software_architecture_visual.html`** 内容对应：在 **VS Code / Cursor** 安装 Mermaid 预览插件后打开本文件可看图；或直接 **双击 HTML**（需联网加载 Mermaid CDN）在浏览器中查看。

各模块函数与职责的完整说明：**[`docs/function_reference.md`](function_reference.md)**

详见文字版分层说明：`docs/app_mvvm_architecture.md`。

---

## 1. 分层总架构

```mermaid
flowchart TB
    subgraph Persist["持久化 / 资源"]
        JSON["config/robot_mdh_template.json"]
        STL["assets/ur5e/collision/*.stl（可选）"]
        OUT["outputs/（导出结果）"]
    end

    subgraph View["视图层 qt_app/views + widgets"]
        MW["MainWindow"]
        CANVAS["RobotCanvas（Matplotlib 3D）"]
        JPC["JointPlotCanvas"]
        MW --> CANVAS
        MW --> JPC
    end

    subgraph VM["视图模型 SimulationViewModel"]
        SVM["build_config / validate / 调用 Service / 摘要"]
    end

    subgraph SVC["应用服务 python/services"]
        SIMS["SimulationService"]
        TRS["TrajectoryService"]
        LANG["LanguageTaskService"]
        PRG["RobotProgramService"]
    end

    subgraph Domain["领域核心 python/"]
        ROB["SixDofRobot"]
        SIM["simulator.py / trajectory.py"]
    end

    JSON -->|"启动 load"| SVM
    MW -->|"link 表格 / q_init / steps -> config dict"| SVM
    SVM --> SIMS
    SVM --> TRS
    SVM --> LANG
    SVM --> PRG
    SIMS --> SIM
    SIMS --> ROB
    TRS --> ROB
    TRS --> SIM
    CANVAS -.-> STL
    CANVAS --> ROB
    SIMS --> OUT
```

---

## 2. 完整变量与数据传递（View / ViewModel / Model）

本节把 **控件读数 → ViewModel → Service / SixDofRobot → 画布与文本** 上的实际变量名与字典键集中说明；对齐 `MainWindow._build_active_config`、`_wrap_track_result`、`TrajectoryService.run_commands`。

### 2.1 View 层：`MainWindow` 与数据流相关的状态

| 成员 / 控件 | 类型或产出 | 作用 |
|-------------|-------------|------|
| `viewmodel` | `SimulationViewModel` | Qt 视图与 ViewModel 的分界 |
| `parameter_table`（行收集） | 表格 → `list[dict]` | `_collect_link_rows` → `build_config(..., link_rows, ...)` |
| `q_init_edit` / `q_goal_edit` / `duration_edit` / `steps_edit` | 字符串解析为数值 | 写入 `config["default_state"]`、`config["simulation_defaults"]` |
| 轨迹面板（轴、半径、长度等） | 控件值 | `build_trajectory_command` → `TrajectoryCommand` |
| `command_edit` | `str` | `parse_task_blocks`；可行性定时器 |
| `task_table` | 表格 ⇄ `TaskBlock` | `_collect_task_blocks_from_table` → `run_task_blocks` |
| `current_results` | `Optional[dict]` | `fk_demo`、`ptp`、`cartesian_track`、`active_config` |
| `current_robot` | `Optional[SixDofRobot]` | 与画布一致：`robot_from_config(active_config)` |
| `current_task_blocks` | `list` | 与任务表同步 |
| `animation_result` / `animation_frame` | 快照 + 索引 | 播放消费与 `cartesian_track` 同型数组 |
| `feasibility_label` | `QLabel` | `precheck_task_blocks` + `summarize_precheck` |
| `robot_canvas` / `joint_canvas` / `result_box` | 控件 | **消费端**：robot + results |

### 2.2 ViewModel：`SimulationViewModel`「入参 → 出参」

| 方法 | 典型入参 | 返回值要点 | 消费者 |
|------|----------|------------|--------|
| `build_config` | `link_rows`, `q_*`, `duration_s`, `steps` | `config` | validate → `run_*` |
| `validate_config` | `config` | 无（抛错即失败） | 仿真与轨迹 |
| `robot_from_config` | `config` | `SixDofRobot` | `current_robot` |
| `run_simulation` | `config` | `{ fk_demo, ptp, cartesian_track }` + `active_config` | UI |
| `run_trajectory_command` | `config`, `TrajectoryCommand` | `_wrap_track_result`，顶层键同上 | UI |
| `run_task_blocks` | `config`, `TaskBlock[]` | 同上；`cartesian_track` 可含 `task_blocks` | UI |
| `precheck_task_blocks` | `config`, `TaskBlock[]` | `precheck` | `feasibility_label` |
| `parse_task_blocks` | `str` | `TaskBlock[]` | `current_task_blocks` |
| `_wrap_track_result` | `track_result` | 合并 `run_fk` → `fk_demo`；`ptp`/`cartesian_track`/`active_config` | `current_results` |

### 2.3 端到端示意（payload 在箭头上）

```mermaid
flowchart TB
  subgraph VW["MainWindow(View)"]
    VUI["控件: parameter_table, q/edits,<br/>trajectory 面板, command_edit, task_table"]
    VMEM["状态: current_results<br/>current_robot, current_task_blocks<br/>animation_result, frame"]
    VOUT["展示: feasibility_label<br/>robot_canvas, joint_canvas, result_box"]
  end

  subgraph SVM["SimulationViewModel"]
    BC["build_config + validate"]
    RSIM["run_simulation"]
    RTR["run_trajectory_* / run_task_blocks"]
    WRAP["_wrap_track_result(...)<br/>内调 run_fk 得到 fk_demo"]
    RFC["robot_from_config"]
    PRE["precheck + summarize_precheck"]
    LANG["parse_task_blocks<br/>build_trajectory_command"]
  end

  subgraph SNUM["服务端数值管线"]
    TRS["TrajectoryService.run_commands<br/>SixDofRobot + IK + evaluate_trajectory"]
    SDS["SimulationService.run_default"]
  end

  VUI -->|"link_rows + q_init + q_goal<br/>duration_s , steps"| BC
  BC -->|"config dict"| RSIM
  BC -->|"config"| RTR
  VUI -->|"自然语言 str"| LANG
  VUI -->|"表格行"| LANG
  LANG -->|"TrajectoryCommand"| RTR
  LANG -->|"TaskBlock[]"| RTR
  VUI -->|"config + TaskBlock[]"| PRE
  PRE -->|"摘要 str"| VOUT

  RTR -->|"config + TrajectoryCommand 列表"| TRS
  TRS -->|"track_result"| WRAP
  WRAP -->|"fk_demo, ptp, cartesian_track, active_config"| VMEM

  RSIM -->|"config"| SDS
  SDS -->|"同上顶层键成套结果"| VMEM

  BC -->|"同一套 config"| RFC
  RFC -->|"SixDofRobot"| VMEM
  VMEM -->|"robot , 阵列, metrics"| VOUT
```

### 2.4 `current_results` 顶层键；`cartesian_track`

与 `python/trajectory.evaluate_trajectory`、`trajectory_service.run_commands` 一致；多任务时 ViewModel 在 `cartesian_track` 上追加 `task_blocks`。

**`current_results` 顶层**

| 键 | 说明 |
|----|------|
| `fk_demo` | `SimulationService.run_fk(config)`（轨迹 `_wrap_track_result` 路径） |
| `ptp` | `cartesian_track` 的深拷贝，`summary.trajectory_type` 改写为预览语义 |
| `cartesian_track` | 轨迹主字典：阵列 + `summary` + `metrics` + `precheck` + `commands` 等 |
| `active_config` | 本次运行的 `config` 快照 |

**`cartesian_track` 内（`evaluate_trajectory` + TrajectoryService 增补）**

- **阵列与时间：**`time_s`，`q_rad` / `q_deg`，`position_m`，`desired_position_m`，`rpy_deg`，`transforms`
- **`summary`：**机器人名、`source`、`trajectory_type`、采样数、时间跨度
- **`metrics`：**`max_position_error_m`、`mean_position_error_m`；`ik_failure_count` / `ik_failure_ratio`；`max_jacobian_condition`、`near_singular_count`；关节步长与地面/连杆间隙标量（见 `_joint_motion_metrics`、`_environment_motion_metrics`）
- **命令与诊断：**`precheck`，`commands` / `requested_commands`，`command` / `requested_command`，`segment_diagnostics`
- **多任务：**`task_blocks`（仅 `run_task_blocks`）

---

## 3. `config` 与 `SixDofRobot` 的传递

```mermaid
flowchart LR
    subgraph UI["界面"]
        TBL["parameter_table<br/>六行 DH"]
        EDT["q_init / q_goal / duration / steps"]
    end

    subgraph VM2["SimulationViewModel"]
        BC["build_config"]
        CF["config: dict<br/>deepcopy(_config)<br/>links from table"]
    end

    subgraph Model["Model"]
        LM["load_robot_config(config)"]
        SD["SixDofRobot<br/>self.a_m, d_m,<br/>alpha_rad, base, tool…"]
    end

    TBL --> BC
    EDT --> BC
    BC --> CF
    CF --> LM
    LM --> SD
```

---

## 4. 单条轨迹指令：时序

```mermaid
sequenceDiagram
    participant V as MainWindow
    participant VM as SimulationViewModel
    participant TR as TrajectoryService
    participant R as SixDofRobot
    participant C as RobotCanvas

    V->>VM: build_config(...) 得 config
    V->>VM: build_trajectory_command(...)
    VM->>TR: run_command(config, command)
    TR->>R: SixDofRobot(config)
    Note over TR,R: desired 点序列 - IK 步进 - q_track / position_m
    TR-->>VM: track_result(dict)
    VM-->>V: wrap: fk_demo, ptp, cartesian_track

    V->>VM: robot_from_config(config)
    VM->>R: SixDofRobot(config) 再现实例
    V->>C: update_plot(robot, ptp, cartesian_track)
```

---

## 5. 自然语言多段任务

```mermaid
flowchart LR
    TXT["command_edit 文本"]
    LANG["LanguageTaskService.parse<br/>规则 + 正则"]
    TB["TaskBlock 列表"]
    CMD["TrajectoryCommand<br/>task.to_command()"]
    TRJ["TrajectoryService.run_commands"]

    TXT --> LANG
    LANG --> TB
    TB --> CMD
    CMD --> TRJ
```

---

## 6. 核心数据形态（报告摘录）

| 变量/类型 | 产生于 | 含义 | 主要消费者 |
|-----------|--------|------|------------|
| `config` | JSON + `build_config` | 整份机器人与仿真默认参数 | `SixDofRobot`、各 Service |
| `SixDofRobot` | `SixDofRobot(config)` | FK / 约束 / Jacobian / IK | TrajectoryService、画布 |
| `TrajectoryCommand` | UI 或 `TaskBlock` | 单段轨迹 | TrajectoryService |
| `TaskBlock` | LanguageService | 解析后的子任务 | `run_task_blocks` |
| `cartesian_track` | `evaluate_trajectory` + TrajectoryService 增补 | 时间序列阵列 + `summary` + `metrics` + `precheck` + `commands` 等 | 3D、播放、导出、误差卡片 |
| `results`（bundle） | `run_simulation` / `_wrap_track_result` | `fk_demo`、`ptp`、`cartesian_track`、`active_config` | `current_results` |

---

## 7. 源码索引

- `python/robot_model.py` — `SixDofRobot`
- `python/services/trajectory_service.py`
- `python/services/language_service.py`
- `qt_app/viewmodels/simulation_viewmodel.py`
- `qt_app/views/main_window.py`
- `qt_app/widgets/robot_canvas.py`、`ur5e_visual_model.py`
