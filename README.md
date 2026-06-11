# 六自由度机械臂模型

本项目实现了一个通用 6R 六自由度机械臂的完整运动学仿真流程，覆盖：

- 基于 `Modified DH` 的模型定义
- MATLAB Robotics Toolbox 参考实现
- Python 可调用仿真核心
- `PySide6 + matplotlib` 的桌面交互界面
- 统一的结果导出格式与比对脚本

## 目录说明

- `config/robot_mdh_template.json`：共享 MDH 参数、offset、关节限位、默认初始条件
- `config/result_schema.json`：统一结果结构模板
- `docs/model_spec.md`：MDH 参数表、offset 设计依据
- `docs/result_schema.md`：结果文件结构说明
- `matlab/`：MATLAB 参考模型、FK 和轨迹仿真
- `python/`：Python 运动学模型、轨迹规划、结果导出、比对脚本
- `qt_app/`：Qt 图形界面
- `outputs/`：默认导出的仿真结果

## Python 运行方式

先安装依赖：

```bash
python -m pip install -r requirements.txt
```

运行 Python 仿真核心：

```bash
python -m python.cli
```

运行 Qt 图形界面：

```bash
python -m qt_app.main
```

## MATLAB 运行方式

前提：

- 已安装 Peter Corke Robotics Toolbox for MATLAB
- 已执行 `startup_rvc`

建议运行顺序：

```matlab
cd matlab
run_fk_demo
run_circle_demo
run_spatial_curve_demo
run_line_demo
run_trajectory_demo
```

脚本说明：

- `build_robot_6dof.m`：从共享 JSON 建立 `SerialLink`
- `run_fk_demo.m`：输出默认姿态和目标姿态的末端位姿
- `run_circle_demo.m`：专门演示末端在指定平面画圆，并导出圆轨迹结果
- `run_spatial_curve_demo.m`：专门演示末端跟踪三维螺旋/空间曲线，并导出结果
- `run_line_demo.m`：专门演示末端沿直线运动，并使用前三轴位置、后三轴姿态的分解逆解思路
- `run_trajectory_demo.m`：完成点到点和轨迹跟踪演示，并导出结果
- `export_simulation_results.m`：导出 CSV、JSON、MAT

## Qt 界面功能

Qt 主程序支持：

- 手动编辑每个关节的 `a`、`alpha`、`d`
- 编辑 `offset` 和关节范围 `Q0 ~ Q1`
- 设置初始关节角、目标关节角、仿真时长、采样点数
- 输入或选择轨迹指令，模拟直线、圆和空间曲线运动
- 使用播放、暂停、重置和时间轴查看机械臂逐帧运动过程
- 在 3D 主视图中以简化实体方式显示机械臂：圆柱表示连杆，球体表示关节与 TCP
- 运行点到点和轨迹跟踪仿真
- 查看机械臂最终姿态、末端轨迹和关节角变化
- 导出当前结果

轨迹指令示例：

```text
绕 z轴 画圆 半径=0.03
沿 x轴 画直线 长度=0.08
空间 螺旋 半径=0.04 抬升=0.08 圈数=1.5
```

## MATLAB / Python 结果比对

如果 MATLAB 和 Python 都已经导出了 `pose_trajectory.csv`，可以运行：

```bash
python -m python.compare_results --reference outputs/matlab_ptp/pose_trajectory.csv --candidate outputs/python_ptp/pose_trajectory.csv
```

也可以将路径替换成轨迹跟踪结果文件夹中的 CSV。

## 当前默认模型

当前默认模型已经切换为更适合空间自由运动的 6R 配置：

- 结构：`J1` 底座、`J2` 肩、`J3` 肘、`J4-J6` 球腕风格姿态轴
- 特点：增大前两段臂长、提高底座高度、拉开腕部工作空间，更适合斜平面圆和三维空间曲线
- 默认初始位形：`[41.37, -35.43, 47.2, 43.55, 49.29, 7.18] deg`
- 默认提供空间曲线参数：半径 `0.04 m`、抬升 `0.12 m`、圈数 `1.5`、轴方向 `[1, 1, 1]`

如果你后续拿到真实机械臂参数，只需修改 `config/robot_mdh_template.json`。

## 已完成的本地验证

已验证：

- `python -m python.cli` 可成功运行并导出结果
- Qt 离屏初始化可成功创建主窗口

未自动验证：

- MATLAB 脚本尚未在当前环境直接执行，因为当前环境不提供 MATLAB 运行时
