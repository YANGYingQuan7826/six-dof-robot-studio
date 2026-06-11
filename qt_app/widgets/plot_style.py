from __future__ import annotations

from matplotlib import rcParams


def configure_matplotlib_fonts() -> None:
    rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "Microsoft YaHei UI",
        "SimHei",
        "Noto Sans CJK SC",
        "PingFang SC",
        "Arial Unicode MS",
        "DejaVu Sans",
    ]
    rcParams["axes.unicode_minus"] = False
