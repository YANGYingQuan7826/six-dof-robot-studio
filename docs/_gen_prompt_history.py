# -*- coding: utf-8 -*-
"""Build docs/full_prompt_history_from_project_start.md from Cursor transcript."""
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent
TRANSCRIPT = Path(
    r"C:\Users\77203\.cursor\projects\c-Users-77203-Desktop\agent-transcripts"
    r"\9104a15a-2182-4206-b50b-89f6000466af\9104a15a-2182-4206-b50b-89f6000466af.jsonl"
)
DST = ROOT / "full_prompt_history_from_project_start.md"


def extract_prompts(path: Path) -> list[tuple[int, str]]:
    out: list[tuple[int, str]] = []
    with path.open(encoding="utf-8") as f:
        for jsonl_line, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if obj.get("role") != "user":
                continue
            msg = obj.get("message", {}).get("content", [])
            texts: list[str] = []
            for part in msg:
                if isinstance(part, dict) and part.get("type") == "text":
                    texts.append(part.get("text", ""))
            blob = "\n".join(texts)
            m = re.search(r"<user_query>\s*(.*?)\s*</user_query>", blob, re.DOTALL)
            body = (m.group(1).strip() if m else blob.strip())
            if body:
                out.append((jsonl_line, body))
    return out


def main() -> None:
    prompts = extract_prompts(TRANSCRIPT)
    lines: list[str] = []
    lines.append("# 六自由度机械臂项目：完整用户 Prompt 追溯")
    lines.append("")
    lines.append(
        "本文档由 Cursor 本地会话 transcript **`9104a15a-2182-4206-b50b-89f6000466af`** "
        "解析生成：提取其中 **`role=user`** 且正文含 **`<user_query>...</user_query>`** "
        "的全部条目，按 transcript 出现顺序编号。"
    )
    lines.append("")
    lines.append(
        "> **说明**：不包含编辑器内的纯本地改动；若有其他分支会话，需另行合并对应 transcript。"
    )
    lines.append("")
    lines.append(f"> **条目数**：{len(prompts)}")
    lines.append("")
    lines.append("---")
    lines.append("")
    for idx, (jl, prompt) in enumerate(prompts, 1):
        lines.append(f"## {idx}. （jsonl 行 {jl}）")
        lines.append("")
        lines.extend(prompt.split("\n"))
        lines.append("")
        lines.append("---")
        lines.append("")
    DST.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {DST} ({len(prompts)} prompts, {DST.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
