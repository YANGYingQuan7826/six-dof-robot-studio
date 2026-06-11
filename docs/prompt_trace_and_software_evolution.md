# 需求记录与软件/文档演进说明

本文档根据**多轮对话中的用户指令**与当前仓库内**可追溯的产出物**整理而成，用于说明「在哪些需求驱动下对哪些材料做了哪些修改」。**应用源代码**（如 `qt_app/`、`python/`）中除下述明确提及的联调或文档引用外，不在此逐项展开；若存在仅发生于本地的操作而未纳入版本库，本文亦无法收录。

**全量原始 Prompt：** 若需要「从建立本项目文件夹以来的每一次聊天输入」的**逐字完整列表**（按 Cursor transcript 顺序，当前 **149** 条 `<user_query>`），请参阅 [`full_prompt_history_from_project_start.md`](full_prompt_history_from_project_start.md)。数据源会话：[机械臂项目主对话](9104a15a-2182-4206-b50b-89f6000466af)。可用 `docs/_gen_prompt_history.py` 重新生成 Markdown。

**可视化 HTML：** 若希望在浏览器中查看「用户输入 ↔ 助手改动摘要」并按任务阶段分组（长输入可折叠），请打开 [`prompt_history_visual.html`](prompt_history_visual.html)；可用 `docs/build_prompt_history_html.py` 在 transcript 更新后重新生成。

---

## 1 文档层面的形成过程（按需求主题归纳）

### 1.1 架构可视化：变量与 MVVM 数据传递

**需求概要：** 在架构可视化文档中补充本 App **完整的变量及数据传递示意图**，覆盖 View、ViewModel、Model（含 Service），并与实现对齐。

**主要修改（简要）：**

- 更新 `docs/software_architecture_visual.html` 与 `docs/software_architecture_visual.md`：新增「完整变量与数据传递」章节；补充 **MainWindow** 侧状态、`SimulationViewModel` 方法入参/出参表、端到端 **Mermaid** 流程图、`current_results` / `cartesian_track` 键级说明；扩展核心数据形态表（含 `results` bundle）；其后章节编号顺延。
- 与 `TrajectoryService.run_commands`、`evaluate_trajectory`、`_wrap_track_result` 等字段保持一致性核对。

---

### 1.2 全量函数说明

**需求概要：** 为本软件补充**所有函数**说明，并详细描述函数作用。

**主要修改（简要）：**

- 新增 `docs/function_reference.md`：按模块（`python/`、`qt_app/`、`scripts/`）列出主要类与函数（含若干内部 `_` 方法簇说明），表格化职责说明。
- 在 `docs/app_mvvm_architecture.md`、`docs/software_architecture_visual.md` / `.html` 中增加指向该索引的链接，便于交叉检索。

---

### 1.3 设计与构造报告：提纲 + 正文执行

**需求概要：** 撰写一份**详细报告**，说明软件构造逻辑与思想、构造过程、架构，并**详细解释** Model、ViewModel、View 之间的数据传递；先列详细提纲，再按计划执行。

**主要修改（简要）：**

- 新增并逐步充实 `docs/studio_design_report.md`：前半为多级提纲，后半为按提纲展开的正文（引言、目标、分层、研制过程、架构、各层职责、数据传递专章等）。

---

### 1.4 报告篇幅与可读性取向（面向非编程读者）

**需求概要：** 报告需**非常详细**（约 **10000 字**量级汉字），使**完全没有代码基础**的读者亦能阅读。

**主要修改（简要）：**

- 对 `docs/studio_design_report.md` 进行大幅扩写：增加先导说明、分步场景化叙述、FAQ、演示口述剧本、答辩提要、术语「人话」解释等；控制总汉字数约达万字量级（以工具统计为准）。
- 措辞当时偏教学化、含较多类比与口语结构（后在后续需求中再做收缩，见 1.6）。

---

### 1.5 Markdown 渲染为 HTML，并嵌入既有图示

**需求概要：** 将报告**渲染成 HTML**，并把**此前绘制**的架构/数据流图纳入同一页面。

**主要修改（简要）：**

- 新增 `docs/build_studio_report_html.py`：将 `studio_design_report.md` 转为 HTML（Python `markdown` 库），抽取正文中的 **Mermaid** 块并嵌入 `<div class="mermaid">`；样式与 CDN 初始化对齐 `software_architecture_visual.html`。
- 生成 `docs/studio_design_report.html`；文末附录嵌入与 `software_architecture_visual.html` **同源**的多幅 Mermaid（分层架构、端到端传递、`config`→`SixDofRobot`、单轨迹时序、自然语言任务链等）。
- 在 `studio_design_report.md` 文首补充 HTML 路径与重建命令说明。

---

### 1.6 文体调整为严谨学术表述

**需求概要：** 全文采用**严谨、标准、客观**的学术语言整理；**减少大量类比**；逻辑清楚；尽量以**整段话**陈述。

**主要修改（简要）：**

- 将 `docs/studio_design_report.md` **重写为陈述性学术体**：删减口语化与类比段落；弱化碎片化列表；合并 FAQ/演示话术为「演示流程要点、常见问题与架构陈述提要」等正式小节；术语表改为连贯定义段落；篇幅收敛（正文汉字显著少于万字扩写版）；摘要说明中注明可按教务要求再增补。
- 意图同步更新 `studio_design_report.html` 时宜重新运行 `python docs/build_studio_report_html.py`（若本地尚未执行，HTML 可能与 MD 不完全一致）。

---

### 1.7 本文档（需求—变更总表）

**需求概要：** 将所有 **prompt** 及输入后的修改（简要）整理成一个文档，说明整个软件的**形成过程**。

**主要修改（简要）：**

- 新增本文件 `docs/prompt_trace_and_software_evolution.md`。

---

## 2 与应用代码相关的补充说明（对话摘要）

下列条目来自更早一轮对话摘要中与 **Qt 界面**相关的收尾工作，对应改动主要在 `qt_app/views/main_window.py` 及周边样式，而非 `docs/`：

- **界面布局与轨迹输入区 / 播放区**：重构左侧轨迹输入与右侧模型下方播放区域；优化按钮与卡片样式。
- **语法与静态检查**：对修改过的 UI 代码做校验以减少错误。

上述内容未在本仓库的 `docs/` 列表中单独立档；若课程报告需引用，请以 **Git 提交记录**或 IDE 本地历史为准。

---

## 3 当前 `docs/` 目录中与「软件形成」直接相关的文件一览

| 文件 | 角色 |
|------|------|
| `software_architecture_visual.md` / `.html` | 分层架构与数据流可视化（含变量传递扩展） |
| `function_reference.md` | 函数/方法索引 |
| `app_mvvm_architecture.md` | MVVM 文字说明与外链 |
| `studio_design_report.md` | 设计与构造报告（学术版正文） |
| `studio_design_report.html` | 报告 HTML + Mermaid 附录图 |
| `build_studio_report_html.py` | MD→HTML 生成脚本 |
| `model_spec.md` / `result_schema.md` | 模型与结果 schema（若课程沿用） |
| `prompt_trace_and_software_evolution.md` | **本文**：需求—变更追溯 |

---

## 4 使用建议

- **答辩或归档**：可将本文作为「文档需求与迭代说明」附录，与 `studio_design_report.md` 并列提交。
- **保持同步**：今后若有新需求驱动文档或脚本变更，建议在本文**第 1 节**末尾按相同格式追加条目，并标注日期或版本。

---

*文档结束。*
