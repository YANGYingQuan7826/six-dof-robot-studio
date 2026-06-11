from __future__ import annotations

import argparse
from pathlib import Path

from .robot_model import default_config_path
from .simulator import run_default_simulation


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the 6-DOF manipulator kinematics simulator.")
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config_path(),
        help="Path to the shared robot MDH JSON config.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "outputs",
        help="Folder used to store JSON and CSV outputs.",
    )
    args = parser.parse_args()

    results = run_default_simulation(args.config, args.output)
    fk_pose = results["fk_demo"]["initial_pose"]
    print("Simulation finished.")
    print(f"Initial end-effector position (m): {fk_pose['position_m']}")
    print(f"Initial end-effector RPY (deg): {fk_pose['rpy_deg']}")
    print(f"Output folder: {args.output}")


if __name__ == "__main__":
    main()
