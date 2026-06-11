from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def load_pose_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)

    return {
        "time_s": np.array([float(row["time_s"]) for row in rows], dtype=float),
        "position_m": np.array([[float(row["x_m"]), float(row["y_m"]), float(row["z_m"])] for row in rows], dtype=float),
        "rpy_deg": np.array(
            [[float(row["roll_deg"]), float(row["pitch_deg"]), float(row["yaw_deg"])] for row in rows],
            dtype=float,
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare two exported pose trajectory CSV files.")
    parser.add_argument("--reference", type=Path, required=True, help="Reference pose_trajectory.csv")
    parser.add_argument("--candidate", type=Path, required=True, help="Candidate pose_trajectory.csv")
    args = parser.parse_args()

    reference = load_pose_csv(args.reference)
    candidate = load_pose_csv(args.candidate)

    if len(reference["time_s"]) != len(candidate["time_s"]):
        raise ValueError("两份结果的采样点数不一致。")

    position_error = np.linalg.norm(reference["position_m"] - candidate["position_m"], axis=1)
    rpy_error = np.linalg.norm(reference["rpy_deg"] - candidate["rpy_deg"], axis=1)

    print("Result comparison finished.")
    print(f"Samples: {len(position_error)}")
    print(f"Max position difference (m): {np.max(position_error):.6f}")
    print(f"Mean position difference (m): {np.mean(position_error):.6f}")
    print(f"Max RPY difference (deg): {np.max(rpy_error):.6f}")
    print(f"Mean RPY difference (deg): {np.mean(rpy_error):.6f}")


if __name__ == "__main__":
    main()
