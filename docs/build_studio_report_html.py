"""Generate docs/studio_design_report.html from studio_design_report.md + diagram appendix."""
from __future__ import annotations

import pathlib
import re

import markdown

ROOT = pathlib.Path(__file__).resolve().parent
MD_PATH = ROOT / "studio_design_report.md"
OUT_PATH = ROOT / "studio_design_report.html"

APPENDIX_MERMAIDS: list[tuple[str, str]] = [
    (
        "图 A · 分层总架构（持久化 / View / ViewModel / Service / 领域核心）",
        """flowchart TB
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
    SIMS --> OUT""",
    ),
    (
        "图 B · View — ViewModel — Service 端到端数据传递（payload 示意）",
        """flowchart TB
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
  VMEM -->|"robot , 阵列, metrics"| VOUT""",
    ),
    (
        "图 C · 配置 config 与 SixDofRobot 的传递",
        """flowchart LR
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
    LM --> SD""",
    ),
    (
        "图 D · 单条轨迹指令：时序（谁在什么时刻交换什么）",
        """sequenceDiagram
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
    V->>C: update_plot(robot, ptp, cartesian_track)""",
    ),
    (
        "图 E · 自然语言 → TaskBlock → TrajectoryService（离线规则）",
        """flowchart LR
    TXT["command_edit 文本"]
    LANG["LanguageTaskService.parse<br/>规则 + 正则"]
    TB["TaskBlock 列表"]
    CMD["TrajectoryCommand<br/>task.to_command()"]
    TRJ["TrajectoryService.run_commands"]

    TXT --> LANG
    LANG --> TB
    TB --> CMD
    CMD --> TRJ""",
    ),
]


HEAD = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>六自由度机械臂 Studio · 设计与构造报告（HTML）</title>
  <script src="https://cdn.jsdelivr.net/npm/mermaid@10.6.1/dist/mermaid.min.js"></script>
  <script>
    mermaid.initialize({
      startOnLoad: true,
      theme: "base",
      themeVariables: {
        primaryColor: "#e0e7ff",
        primaryTextColor: "#111827",
        primaryBorderColor: "#6366f1",
        lineColor: "#475569",
        secondaryColor: "#f1f5f9",
        tertiaryColor: "#f8fafc"
      },
      flowchart: { useMaxWidth: true, htmlLabels: true, curve: "basis" },
      sequence: { useMaxWidth: true }
    });
  </script>
  <style>
    :root { color-scheme: light; }
    body {
      font-family: "Segoe UI", "Microsoft YaHei UI", "PingFang SC", sans-serif;
      max-width: 920px;
      margin: 0 auto;
      padding: 24px 18px 56px;
      line-height: 1.72;
      color: #1f2937;
      background: linear-gradient(180deg, #fafbfc 0%, #eef2ff55 100%);
    }
    h1 { font-size: 1.65rem; margin-bottom: 0.35rem; color: #111827; }
    h2 { font-size: 1.22rem; margin-top: 2.1rem; color: #374151; border-bottom: 2px solid #c7d2fe; padding-bottom: 6px; }
    h3 { font-size: 1.05rem; margin-top: 1.35rem; color: #4b5563; }
    p { margin: 0.65rem 0; }
    ul, ol { padding-left: 1.35rem; }
    .muted { color: #6b7280; font-size: 0.92rem; }
    .banner {
      background: #1e293b;
      color: #e2e8f0;
      padding: 12px 16px;
      border-radius: 10px;
      margin: 16px 0;
      font-size: 0.9rem;
    }
    .banner strong { color: #93c5fd; }
    .mermaid {
      background: #ffffff;
      border: 1px solid #e2e8f0;
      border-radius: 12px;
      padding: 20px 12px;
      margin: 20px 0;
      overflow-x: auto;
      box-shadow: 0 1px 3px rgba(15,23,42,0.06);
      font-family: ui-monospace, monospace;
      white-space: pre-wrap;
    }
    table {
      border-collapse: collapse;
      width: 100%;
      margin: 12px 0;
      font-size: 0.9rem;
    }
    th, td {
      border: 1px solid #e5e7eb;
      padding: 8px 10px;
      text-align: left;
      vertical-align: top;
    }
    th { background: #f8fafc; font-weight: 600; }
    code { background: #f1f5f9; padding: 2px 6px; border-radius: 4px; font-size: 0.88em; }
    pre {
      background: #1e293b;
      color: #e2e8f0;
      padding: 14px 16px;
      border-radius: 10px;
      overflow-x: auto;
      font-size: 0.86rem;
    }
    pre code { background: transparent; color: inherit; padding: 0; }
    blockquote {
      border-left: 4px solid #a5b4fc;
      margin: 1rem 0;
      padding: 0.4rem 0 0.4rem 1rem;
      color: #4b5563;
      background: #f8fafc;
      border-radius: 0 8px 8px 0;
    }
    hr.appendix-split {
      margin: 3rem 0 2rem;
      border: none;
      border-top: 2px dashed #cbd5e1;
    }
    .appendix-head { color: #4338ca; }
  </style>
</head>
<body>
"""


def md_to_html_fragment(text: str) -> str:
    mermaid_blocks: list[str] = []

    def repl(m: re.Match[str]) -> str:
        mermaid_blocks.append(m.group(1).strip())
        i = len(mermaid_blocks) - 1
        return f"\n\n<!-- MERMAID_SLOT_{i} -->\n\n"

    cleaned = re.sub(r"```mermaid\n([\s\S]*?)```", repl, text)

    fragment = markdown.markdown(
        cleaned,
        extensions=[
            "markdown.extensions.tables",
            "markdown.extensions.fenced_code",
            "markdown.extensions.nl2br",
            "markdown.extensions.sane_lists",
        ],
    )

    for i, body in enumerate(mermaid_blocks):
        slot = f"<!-- MERMAID_SLOT_{i} -->"
        div = f'<div class="mermaid">\n{body}\n</div>'
        fragment = fragment.replace(f"<p>{slot}</p>", div)
        fragment = fragment.replace(slot, div)

    return fragment


def build_appendix() -> str:
    parts: list[str] = [
        '<hr class="appendix-split" />',
        '<h2 class="appendix-head">附录：架构与数据流图解</h2>',
        '<p class="muted">以下图示与 <code>docs/software_architecture_visual.html</code> 同源，便于在「设计报告」单页内对照阅读；需<strong>联网</strong>加载 Mermaid CDN 才能渲染。</p>',
    ]
    for title, graph in APPENDIX_MERMAIDS:
        parts.append(f"<h3>{title}</h3>")
        parts.append(f'<div class="mermaid">\n{graph}\n</div>')
    parts.append(
        '<p class="muted">更完整的变量表与数据键说明仍见 <a href="software_architecture_visual.html">software_architecture_visual.html</a>。</p>'
    )
    return "\n".join(parts)


def main() -> None:
    md = MD_PATH.read_text(encoding="utf-8")
    body_inner = md_to_html_fragment(md)
    appendix = build_appendix()

    out = "".join(
        [
            HEAD,
            '  <h1>六自由度机械臂仿真 Studio · 设计与构造报告</h1>\n',
            '  <p class="muted">由 <code><a href="studio_design_report.md">studio_design_report.md</a></code> 自动生成。'
            " 含正文 Mermaid 与《架构与数据流》同源附录图。</p>\n",
            '  <div class="banner"><strong>如何查看：</strong> 使用 Chrome / Edge 打开本 HTML；若图不显示，请检查网络能否访问 jsdelivr CDN。</div>\n',
            '  <article class="report-main">\n',
            body_inner,
            "\n  </article>\n",
            '  <section class="appendix-section">\n',
            appendix,
            "\n  </section>\n",
            '  <p class="muted" style="margin-top:2.5rem;">HTML 可通过 <code>docs/build_studio_report_html.py</code> 重新生成。</p>\n',
            "</body>\n</html>\n",
        ]
    )

    OUT_PATH.write_text(out, encoding="utf-8")
    print(f"Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
