# -*- coding: utf-8 -*-
"""从 Cursor transcript 生成「用户提问 vs 助手改动摘要」可视化 HTML。"""
from __future__ import annotations

import html as html_lib
import json
import re
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
TRANSCRIPT = Path(
    r"C:\Users\77203\.cursor\projects\c-Users-77203-Desktop\agent-transcripts"
    r"\9104a15a-2182-4206-b50b-89f6000466af\9104a15a-2182-4206-b50b-89f6000466af.jsonl"
)
PROJECT_MARKER = "六自由度机械臂模型"
HTML_OUT = ROOT / "prompt_history_visual.html"

# 折叠阈值：超过则展示摘要 + <details>
_COLLAPSE_CHARS = 360
_COLLAPSE_LINES = 10

_ADD_FILE_RE = re.compile(r"\*\*\*\s*Add File:\s*(.+?)\s*\r?\n", re.MULTILINE)
_UPD_FILE_RE = re.compile(r"\*\*\*\s*Update File:\s*(.+?)\s*\r?\n", re.MULTILINE)

PHASE_ORDER = [
    "立项需求（总目标）",
    "启动实施（按计划批量落地）",
    "规划澄清与早期迭代",
    "模型参数 / DH·MDH / 配置",
    "MATLAB / RTB 仿真脚本",
    "Python 运动学与仿真核心",
    "Qt / MVVM 与可视化界面",
    "缺陷排查与报错修复",
    "打包与分发",
    "文档 / 课程报告 / 架构说明",
    "国际化与文案",
    "其它交流与迭代",
]


def _extract_user_body(user_obj: dict) -> str | None:
    msg = user_obj.get("message", {}).get("content", [])
    texts: list[str] = []
    for part in msg:
        if isinstance(part, dict) and part.get("type") == "text":
            texts.append(part.get("text", ""))
    blob = "\n".join(texts)
    m = re.search(r"<user_query>\s*(.*?)\s*</user_query>", blob, re.DOTALL)
    body = (m.group(1).strip() if m else blob.strip())
    return body or None


def _to_rel_project_path(abs_path: str) -> str:
    s = abs_path.replace("/", "\\")
    if PROJECT_MARKER in s:
        idx = s.index(PROJECT_MARKER) + len(PROJECT_MARKER)
        tail = s[idx:].lstrip("\\").replace("\\", "/")
        return tail or "(项目根)"
    parts = Path(s.replace("\\", "/")).parts
    return "/".join(parts[-4:]) if parts else abs_path


def _summarize_assistant_messages(assistants: list[dict]) -> tuple[list[str], list[str], list[str]]:
    """返回 (新增文件, 修改文件, 其它要点)。"""
    added: list[str] = []
    updated: list[str] = []
    misc: list[str] = []

    def push_misc(line: str) -> None:
        line = line.strip()
        if line and line not in misc:
            misc.append(line)

    for obj in assistants:
        for part in obj.get("message", {}).get("content", []):
            if not isinstance(part, dict) or part.get("type") != "tool_use":
                continue
            name = part.get("name") or ""
            inp = part.get("input")

            if name == "ApplyPatch":
                patch = inp if isinstance(inp, str) else str(inp or "")
                for m in _ADD_FILE_RE.finditer(patch):
                    added.append(_to_rel_project_path(m.group(1).strip()))
                for m in _UPD_FILE_RE.finditer(patch):
                    updated.append(_to_rel_project_path(m.group(1).strip()))

            elif name == "StrReplace" and isinstance(inp, dict):
                pth = inp.get("path") or inp.get("file")
                if pth:
                    updated.append(_to_rel_project_path(str(pth)))

            elif name == "Write" and isinstance(inp, dict):
                pth = inp.get("path") or inp.get("file_path")
                if pth:
                    added.append(_to_rel_project_path(str(pth)))

            elif name == "Delete" and isinstance(inp, dict):
                pth = inp.get("path")
                if pth:
                    push_misc(f"删除文件：{_to_rel_project_path(str(pth))}")

            elif name == "Shell" and isinstance(inp, dict):
                desc = inp.get("description") or ""
                cmd = (inp.get("command") or "")[:72]
                if desc:
                    push_misc(f"终端：{desc}")
                elif cmd:
                    push_misc(f"终端命令片段：`{cmd}`")

            elif name == "Glob":
                push_misc("工作区内 Glob 扫描文件")

            elif name == "Grep":
                push_misc("代码文本搜索（Grep）")

            elif name == "ReadFile" or name == "Read":
                push_misc("读取参考文件（只读）")

            elif name == "Subagent":
                if isinstance(inp, dict):
                    push_misc(f"子代理探索：{inp.get('description', '只读任务')}")

            elif name == "AskQuestion":
                push_misc("向用户发起选项确认（AskQuestion）")

            elif name == "CreatePlan":
                push_misc("生成结构化实施计划（CreatePlan）")

            elif name == "TodoWrite":
                push_misc("更新任务清单状态（TodoWrite）")

            else:
                push_misc(f"工具调用：{name}")

    def _uniq(xs: list[str]) -> list[str]:
        seen: set[str] = set()
        out: list[str] = []
        for x in xs:
            if x not in seen:
                seen.add(x)
                out.append(x)
        return out

    return _uniq(added), _uniq(updated), misc


def _first_meaningful_line(text: str) -> str:
    for ln in text.splitlines():
        s = ln.strip()
        if len(s) >= 8:
            return s
    return text.strip().split("\n", 1)[0][:120]


def _user_preview_full(text: str) -> tuple[str, str | None]:
    """(可见摘要, 折叠全文或 None)"""
    lines = text.split("\n")
    n_lines = len(lines)
    if len(text) <= _COLLAPSE_CHARS and n_lines <= _COLLAPSE_LINES:
        return text, None
    preview_lines = lines[: min(5, n_lines)]
    preview = "\n".join(preview_lines).strip()
    if len(preview) > 320:
        preview = preview[:317].rstrip() + "…"
    return preview, text


def path_phase_hint(paths: list[str]) -> str | None:
    """当用户话术无明显关键词时，用本轮 touched 路径兜底归类。"""
    ps = [p.replace("\\", "/").lower() for p in paths]

    def has_prefix(pref: str) -> bool:
        return any(x.startswith(pref) for x in ps)

    if has_prefix("qt_app/"):
        return "Qt / MVVM 与可视化界面"
    if any("release/" in x for x in ps):
        return "打包与分发"
    if has_prefix("matlab/"):
        return "MATLAB / RTB 仿真脚本"
    if has_prefix("python/"):
        return "Python 运动学与仿真核心"
    if has_prefix("config/"):
        return "模型参数 / DH·MDH / 配置"
    if has_prefix("docs/"):
        return "文档 / 课程报告 / 架构说明"
    return None


def classify_phase(prompt: str, turn_index: int, touched_paths: list[str]) -> str:
    p = prompt
    low = p.lower()

    if turn_index == 0 or "我现在要实现以下内容" in p:
        return "立项需求（总目标）"

    if "implement the plan" in low or ("do not edit the plan file" in low):
        return "启动实施（按计划批量落地）"

    if any(
        k in p
        for k in (
            "报告",
            "提纲",
            "架构",
            "函数说明",
            "演进",
            "追溯",
            "prompt",
            "documentation",
            "mvvm",
            "software_architecture",
            "studio_design",
            "可视化文档",
            "架构图",
            "交叉引用",
            "html文件",
            "渲染成html",
            "prompt_history",
            "追溯文档",
        )
    ):
        return "文档 / 课程报告 / 架构说明"

    if any(k in p for k in ("中英文", "翻译", "国际化", "language", "English", "中文界面")):
        return "国际化与文案"

    if any(k in low for k in ("release", "打包", "分发", "exe")):
        return "打包与分发"

    if any(
        k in p
        for k in ("报错", "错误", "traceback", "失败", "解决不了", "异常", "warning", "fix")
    ):
        return "缺陷排查与报错修复"

    if any(k in low for k in ("qt", "pyside", "界面", "布局", "按钮", "画布", "matplotlib")):
        return "Qt / MVVM 与可视化界面"

    if any(k in low for k in ("matlab", "rtb", "serial", "run_trajectory", ".m脚本")):
        return "MATLAB / RTB 仿真脚本"

    if any(k in low for k in ("python", "robot_model", "逆运动", "逆解", "numpy")):
        return "Python 运动学与仿真核心"

    if (
        "MDH" in p
        or "DH" in p
        or "mdh" in low
        or "dh参数" in low
        or "关节角" in p
        or "json" in low and "配置" in p
    ):
        return "模型参数 / DH·MDH / 配置"

    fb = path_phase_hint(touched_paths)
    if fb:
        return fb

    if turn_index <= 6:
        return "规划澄清与早期迭代"

    return "其它交流与迭代"


def _render_assistant_section(
    added: list[str], updated: list[str], misc: list[str],
) -> str:
    chunks: list[str] = []
    if added:
        li = "".join(f"<li><code>{html_lib.escape(a)}</code></li>" for a in added[:80])
        chunks.append(f"<p><strong>新增</strong></p><ul class='files'>{li}</ul>")
    if updated:
        li = "".join(f"<li><code>{html_lib.escape(u)}</code></li>" for u in updated[:80])
        chunks.append(f"<p><strong>修改</strong></p><ul class='files'>{li}</ul>")
    extra = [m for m in misc if m]
    misc_cap = 14
    if extra:
        lim = extra[:misc_cap]
        li = "".join(f"<li>{html_lib.escape(x)}</li>" for x in lim)
        more = ""
        if len(extra) > misc_cap:
            more = f"<p class='muted'>… 另有 {len(extra) - misc_cap} 条动作摘要略</p>"
        chunks.append(f"<p><strong>其它动作</strong></p><ul class='misc'>{li}</ul>{more}")
    if not chunks:
        return "<p class='muted'>（本回合 transcript 中未解析到文件级补丁；可能仅为说明、确认或只读探索）</p>"
    return "\n".join(chunks)


def _render_user_section(prompt: str) -> str:
    headline = html_lib.escape(_first_meaningful_line(prompt))
    preview, full = _user_preview_full(prompt)

    if full is None:
        preview_esc = html_lib.escape(prompt)
        return (
            "<div class='user-main'>"
            f"<p class='headline'>{headline}</p>"
            f"<pre class='prompt'>{preview_esc}</pre>"
            "</div>"
        )

    preview_esc = html_lib.escape(preview)
    full_esc = html_lib.escape(full)
    return (
        "<div class='user-main'>"
        f"<p class='headline'>{headline}</p>"
        "<p class='preview-note'>要点预览（节选）：</p>"
        f"<pre class='prompt preview'>{preview_esc}</pre>"
        "<details class='prompt-details'>"
        "<summary>展开完整用户输入 ▼</summary>"
        f"<pre class='prompt full'>{full_esc}</pre>"
        "</details>"
        "</div>"
    )


def build_html() -> str:
    rows: list[tuple[int, int, dict]] = []
    with TRANSCRIPT.open(encoding="utf-8") as f:
        buf: list[tuple[int, dict]] = []
        for jsonl_line, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            buf.append((jsonl_line, obj))

    i = 0
    turn_no = 0
    while i < len(buf):
        jl, obj = buf[i]
        if obj.get("role") != "user":
            i += 1
            continue
        body = _extract_user_body(obj)
        if not body:
            i += 1
            continue
        assistants: list[dict] = []
        i += 1
        while i < len(buf) and buf[i][1].get("role") == "assistant":
            assistants.append(buf[i][1])
            i += 1
        added, updated, misc = _summarize_assistant_messages(assistants)
        touched = added + updated
        phase = classify_phase(body, turn_no, touched)
        rows.append(
            (
                turn_no,
                jl,
                {
                    "prompt": body,
                    "phase": phase,
                    "added": added,
                    "updated": updated,
                    "misc": misc,
                },
            )
        )
        turn_no += 1

    by_phase: dict[str, list[tuple[int, int, dict]]] = defaultdict(list)
    for turn_no, jl, data in rows:
        by_phase[data["phase"]].append((turn_no, jl, data))

    slug_map: dict[str, str] = {}
    slug_i = 0
    for ph in PHASE_ORDER:
        if ph in by_phase:
            slug_map[ph] = f"phase-{slug_i}"
            slug_i += 1
    for ph in sorted(p for p in by_phase if p not in PHASE_ORDER):
        slug_map[ph] = f"phase-{slug_i}"
        slug_i += 1

    toc_li = []
    for ph in PHASE_ORDER:
        if ph not in by_phase:
            continue
        pid = slug_map[ph]
        toc_li.append(f'<li><a href="#{pid}">{html_lib.escape(ph)}</a> '
                      f'<span class="count">({len(by_phase[ph])})</span></li>')
    other_phases = [p for p in by_phase if p not in PHASE_ORDER]
    for ph in sorted(other_phases):
        pid = slug_map[ph]
        toc_li.append(f'<li><a href="#{pid}">{html_lib.escape(ph)}</a> '
                      f'<span class="count">({len(by_phase[ph])})</span></li>')

    sections_html: list[str] = []

    def emit_phase(ph: str) -> None:
        if ph not in by_phase:
            return
        pid = slug_map[ph]
        cards = []
        for turn_no, jl, data in by_phase[ph]:
            uid = f"t{turn_no}"
            cards.append(
                f'<article class="card" id="{uid}">'
                '<header class="card-head">'
                f"<span class='badge'>#{turn_no + 1}</span>"
                f"<span class='meta'>jsonl 行 {jl}</span>"
                "</header>"
                '<div class="cols">'
                '<section class="col user-col">'
                "<h3>用户输入</h3>"
                f"{_render_user_section(data['prompt'])}"
                "</section>"
                '<section class="col bot-col">'
                "<h3>助手回应（改动摘要）</h3>"
                f"{_render_assistant_section(data['added'], data['updated'], data['misc'])}"
                "</section>"
                "</div>"
                "</article>"
            )
        sections_html.append(
            f'<section class="phase" id="{html_lib.escape(pid)}">'
            f"<h2>{html_lib.escape(ph)}</h2>"
            + "\n".join(cards)
            + "</section>"
        )

    for ph in PHASE_ORDER:
        emit_phase(ph)
    for ph in sorted(p for p in by_phase if p not in PHASE_ORDER):
        emit_phase(ph)

    meta_ts = Path(__file__).name
    return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>六自由度机械臂项目 · Prompt 与改动摘要（按阶段）</title>
  <style>
    :root {{
      --bg: #f6f7fb;
      --card: #fff;
      --border: #e2e5ee;
      --text: #1c1f26;
      --muted: #6b7280;
      --accent: #2563eb;
      --user-bg: #eef6ff;
      --bot-bg: #f3faf4;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
      background: var(--bg);
      color: var(--text);
      line-height: 1.55;
    }}
    .wrap {{
      max-width: 1180px;
      margin: 0 auto;
      padding: 28px 18px 80px;
    }}
    header.site {{
      margin-bottom: 28px;
    }}
    header.site h1 {{
      font-size: 1.45rem;
      margin: 0 0 8px;
    }}
    header.site p {{
      margin: 4px 0;
      color: var(--muted);
      font-size: 0.92rem;
    }}
    nav.toc {{
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 14px 18px;
      margin-bottom: 28px;
    }}
    nav.toc h2 {{
      margin: 0 0 10px;
      font-size: 1rem;
    }}
    nav.toc ul {{
      margin: 0;
      padding-left: 20px;
      columns: 2;
      gap: 24px;
    }}
    @media (max-width: 720px) {{
      nav.toc ul {{ columns: 1; }}
    }}
    nav.toc .count {{
      color: var(--muted);
      font-size: 0.85rem;
    }}
    section.phase {{
      margin-bottom: 36px;
    }}
    section.phase > h2 {{
      font-size: 1.15rem;
      border-bottom: 2px solid var(--accent);
      padding-bottom: 6px;
      margin: 0 0 16px;
    }}
    article.card {{
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 16px 18px;
      margin-bottom: 16px;
      box-shadow: 0 1px 2px rgba(15, 23, 42, 0.05);
    }}
    .card-head {{
      display: flex;
      align-items: center;
      gap: 10px;
      margin-bottom: 12px;
      font-size: 0.88rem;
      color: var(--muted);
    }}
    .badge {{
      background: var(--accent);
      color: #fff;
      padding: 2px 10px;
      border-radius: 999px;
      font-weight: 600;
    }}
    .cols {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 18px;
    }}
    @media (max-width: 900px) {{
      .cols {{ grid-template-columns: 1fr; }}
    }}
    section.col h3 {{
      margin: 0 0 8px;
      font-size: 0.95rem;
    }}
    .user-col {{
      background: var(--user-bg);
      border-radius: 10px;
      padding: 12px 14px;
    }}
    .bot-col {{
      background: var(--bot-bg);
      border-radius: 10px;
      padding: 12px 14px;
    }}
    .headline {{
      font-weight: 600;
      margin: 0 0 8px;
      font-size: 0.98rem;
    }}
    pre.prompt {{
      margin: 0;
      white-space: pre-wrap;
      word-break: break-word;
      font-size: 0.82rem;
      font-family: ui-monospace, "Cascadia Mono", Consolas, monospace;
      background: rgba(255,255,255,0.65);
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 10px;
      max-height: 420px;
      overflow: auto;
    }}
    pre.prompt.preview {{
      max-height: 220px;
    }}
    details.prompt-details {{
      margin-bottom: 10px;
      border: 1px dashed var(--border);
      border-radius: 8px;
      padding: 8px 10px;
      background: rgba(255,255,255,0.5);
    }}
    details.prompt-details summary {{
      cursor: pointer;
      font-weight: 600;
      color: var(--accent);
      user-select: none;
    }}
    .preview-note {{
      margin: 8px 0 4px;
      font-size: 0.8rem;
      color: var(--muted);
    }}
    ul.files, ul.misc {{
      margin: 6px 0 0;
      padding-left: 18px;
      font-size: 0.86rem;
    }}
    ul.files code {{
      font-size: 0.82rem;
      background: rgba(255,255,255,0.85);
      padding: 1px 5px;
      border-radius: 4px;
    }}
    p.muted {{
      color: var(--muted);
      font-size: 0.88rem;
    }}
    a {{ color: var(--accent); }}
  </style>
</head>
<body>
  <div class="wrap">
    <header class="site">
      <h1>六自由度机械臂项目 · 对话追溯（HTML）</h1>
      <p>数据源：Cursor transcript <code>9104a15a-2182-4206-b50b-89f6000466af</code>。</p>
      <p>左侧为用户输入（长文可折叠）；右侧由工具调用推断<strong>新增/修改的文件及简要动作</strong>，不含补丁全文。</p>
      <p class="muted">生成脚本：<code>{html_lib.escape(meta_ts)}</code> · 条目数：<strong>{len(rows)}</strong></p>
    </header>
    <nav class="toc">
      <h2>按任务阶段跳转</h2>
      <ul>
        {"".join(toc_li)}
      </ul>
    </nav>
    {"".join(sections_html)}
  </div>
</body>
</html>
"""


def main() -> None:
    html = build_html()
    HTML_OUT.write_text(html, encoding="utf-8")
    print(f"wrote {HTML_OUT} ({HTML_OUT.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
