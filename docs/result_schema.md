# 仿真结果结构说明

本项目要求 MATLAB、Python、Qt 三端输出同一种结果结构，便于互相校验和界面展示。参考模板见 `config/result_schema.json`。

## 顶层字段

### `summary`
描述当前结果集的来源和规模。

- `robot_name`：机器人名称
- `source`：结果来源，例如 `matlab`、`python`、`qt`
- `trajectory_type`：轨迹类型，例如 `ptp` 或 `cartesian_track`
- `time_span_s`：总时长
- `samples`：采样点数

### `joint_trajectory`
按时间存储关节角轨迹。

每个元素包含：

- `time_s`
- `q_deg`
- `q_rad`

### `pose_trajectory`
按时间存储末端位姿。

每个元素包含：

- `time_s`
- `position_m`
- `rpy_deg`
- `transform`

### `metrics`
描述本次仿真的统计量。

- `max_position_error_m`
- `mean_position_error_m`

## CSV 导出建议
为了方便 MATLAB、Python、Qt 与 Excel 互通，建议同时导出两个 CSV：

1. `joint_trajectory.csv`
2. `pose_trajectory.csv`

建议字段如下：

### `joint_trajectory.csv`
- `time_s`
- `q1_deg` 到 `q6_deg`
- `q1_rad` 到 `q6_rad`

### `pose_trajectory.csv`
- `time_s`
- `x_m`
- `y_m`
- `z_m`
- `roll_deg`
- `pitch_deg`
- `yaw_deg`

## Qt 显示建议
Qt 界面直接消费上述结构即可：

- 参数表格区：读取 `config/robot_mdh_template.json`
- 文本结果区：显示 `summary` 和最后一个 `pose_trajectory`
- 曲线区：使用 `joint_trajectory`
- 三维轨迹区：使用 `pose_trajectory.position_m`
