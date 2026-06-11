# App MVVM 架构设计

**函数全书索引**（各类/函数职责说明）：[`docs/function_reference.md`](function_reference.md)

**设计与构造长篇报告**（提纲 + 正文，MVVM 数据传递专章）：[`docs/studio_design_report.md`](studio_design_report.md)

## 目标
本项目的 App 采用轻量 MVVM 结构，目标是让界面、界面状态和运动学算法解耦：

- `Model / Algorithm`：机械臂参数、正逆运动学、轨迹规划、结果评估
- `Service`：把算法包装成面向 App 的用例接口
- `ViewModel`：管理界面状态、参数校验、调用服务、整理结果摘要
- `View`：只负责控件布局、用户交互和数据展示

## 当前分层

| 层级 | 目录/文件 | 职责 |
| --- | --- | --- |
| Model | `python/robot_model.py` | MDH 建模、FK、IK、雅可比、关节位置 |
| Model | `python/trajectory.py` | PTP、圆轨迹、轨迹评估 |
| Service | `python/services/kinematics_service.py` | 对外提供标准 IK 请求/结果接口 |
| Service | `python/services/simulation_service.py` | 对外提供仿真、结果导出、机器人实例创建 |
| Service | `python/services/language_service.py` | 将自然语言拆解为低代码任务块 |
| Service | `python/services/trajectory_service.py` | 将任务块转换为轨迹并执行平滑 IK 跟踪 |
| Service | `python/services/controller_service.py` | 导出机器人程序、虚拟下发、预留真实硬件控制器接口 |
| ViewModel | `qt_app/viewmodels/simulation_viewmodel.py` | 构建配置、校验参数、调用仿真服务、摘要输出 |
| View | `qt_app/views/main_window.py` | 主界面布局、按钮、表格、状态栏 |
| View | `qt_app/widgets/robot_canvas.py` | 机械臂与轨迹三维绘图 |
| View | `qt_app/widgets/joint_plot_canvas.py` | 关节角曲线绘图 |

## MATLAB 算法如何进入软件

MATLAB 中跑通的逆解算法先作为算法原型，迁移到 `python/robot_model.py` 或新的算法模块中。迁移完成后，不直接从界面调用算法函数，而是通过 `KinematicsService` 统一包装：

```python
request = IKRequest(
    target_position_m=[0.4, 0.1, 0.2],
    target_rpy_deg=[0.0, 45.0, 0.0],
    q_seed_deg=[41.37, -35.43, 47.2, 43.55, 49.29, 7.18],
    mode="full_pose",
)
result = KinematicsService(config).solve_inverse(request)
```

ViewModel 只读取 `IKResult`，例如：

- `success`
- `q_solution_deg`
- `position_error_m`
- `orientation_error_deg`
- `message`

这样后续替换 IK 算法时，界面代码不需要改。

## 视觉设计方向

界面采用现代简约风格，参考 Apple Human Interface Guidelines 的通用原则：

- 清晰的视觉层级
- 足够留白
- 浅色背景与白色卡片
- 圆角控件
- 蓝色主操作按钮
- 弱化边框和网格线
- 让内容和结果优先，而不是让控件抢视觉焦点

当前统一样式位于 `qt_app/styles.py`。

## 产品主链路

当前平台按以下流程组织功能：

1. 用户输入自然语言任务，例如“沿 x 轴移动 3cm，然后绕 z 轴画圆半径 1.5cm”。
2. `LanguageTaskService` 将自然语言拆成低代码任务块。
3. 用户可在界面表格中编辑任务块、删除任务块、调整顺序。
4. `TrajectoryService` 将任务块转换为末端轨迹，并用带关节平滑约束的 IK 生成关节序列。
5. 数字孪生界面显示机械臂运动、末端轨迹、关节角变化和误差诊断。
6. `RobotProgramService` 将仿真结果导出为 `robot_program_dry_run.json`。
7. `DryRunController` 执行虚拟下发，未来真实机械臂只需实现同样的控制器接口。

## 未来硬件接口

真实硬件接入不应直接写在界面层，而应实现 `RobotController` 接口：

```python
class RealRobotController:
    controller_name = "RealRobotController"

    def send_program(self, program: dict) -> ControllerReport:
        # 可替换为厂商 SDK、TCP、ROS、Modbus 或串口通信。
        ...
```

接入真实机械臂前必须先补齐安全检查：

- 关节角、速度、加速度限制
- 工具坐标系和实际 TCP 标定
- 工作空间碰撞和越界检查
- 急停、使能、权限和手动确认流程
- 仿真程序到真实控制器指令格式的二次校验
 