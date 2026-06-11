from __future__ import annotations

from pathlib import Path
from urllib.request import urlretrieve


ROOT = Path(__file__).resolve().parent.parent
TARGET_DIR = ROOT / "assets" / "ur5e" / "collision"

BASE_URL = "https://raw.githubusercontent.com/UniversalRobots/Universal_Robots_ROS2_Description/humble/meshes/ur5e/collision"
FILES = [
    "base.stl",
    "shoulder.stl",
    "upperarm.stl",
    "forearm.stl",
    "wrist1.stl",
    "wrist2.stl",
    "wrist3.stl",
]


def main() -> None:
    TARGET_DIR.mkdir(parents=True, exist_ok=True)
    for filename in FILES:
        target = TARGET_DIR / filename
        url = f"{BASE_URL}/{filename}"
        print(f"Downloading {filename} -> {target}")
        urlretrieve(url, target)
    print(f"Done. UR5e collision meshes saved to: {TARGET_DIR}")


if __name__ == "__main__":
    main()
