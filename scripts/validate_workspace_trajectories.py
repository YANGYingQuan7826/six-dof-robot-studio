#!/usr/bin/env python3
"""Batch-check Cartesian line/circle IK tracking for the bundled UR5e config.

Modes:
  --mode fixed    Use GUI default q_init only + TrajectoryService._best_track (fast geometry sweep).
  --mode prod     Use SimulationViewModel.run_trajectory_command (includes auto-start; slower).

Examples:
  python scripts/validate_workspace_trajectories.py --mode fixed
  python scripts/validate_workspace_trajectories.py --mode prod --steps 31
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

import numpy as np

# Repo root on sys.path
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from python.robot_model import SixDofRobot  # noqa: E402
from python.services import TrajectoryCommand, TrajectoryService  # noqa: E402


def frange(lo: float, hi: float, step: float) -> list[float]:
    n = int(round((hi - lo) / step)) + 1
    out = []
    for i in range(max(n, 2)):
        v = round(lo + i * step, 4)
        if v > hi + 1e-9:
            break
        out.append(v)
    if out[-1] < hi - 1e-9:
        out.append(round(hi, 4))
    return out


def _metrics_from_track(
    robot: SixDofRobot,
    desired: np.ndarray,
    q_track: np.ndarray,
    failures: int,
) -> dict[str, float]:
    actual = np.asarray([robot.fkine(q)[:3, 3] for q in q_track], dtype=float)
    err = np.linalg.norm(actual - desired, axis=1)
    dq = np.diff(q_track, axis=0)
    jd = np.rad2deg(np.linalg.norm(dq, axis=1))
    return {
        "failures": float(failures),
        "max_err_m": float(np.max(err)),
        "mean_err_m": float(np.mean(err)),
        "max_joint_step_deg": float(np.max(jd)) if len(jd) else 0.0,
        "failure_ratio": float(failures / max(len(desired) - 1, 1)),
    }


def run_fixed_mode(config: dict, steps: int, line_max: float, circle_max: float, grid: float) -> list[dict]:
    svc = TrajectoryService()
    robot = SixDofRobot(copy.deepcopy(config))
    q_seed = robot.clamp_to_limits(np.deg2rad(np.asarray(config["default_state"]["q_init_deg"], dtype=float)))
    start_position = robot.fkine(q_seed)[:3, 3]
    rows: list[dict] = []

    for axis in ("x", "y", "z"):
        for length in frange(0.05, line_max, grid):
            cmd = TrajectoryCommand("line", axis, radius_m=0.03, length_m=float(length), rise_m=0.02, turns=1.0)
            dp, qt, fc, used = svc._best_track(robot, q_seed, start_position, cmd, steps)
            m = _metrics_from_track(robot, dp, qt, fc)
            rows.append(
                {
                    "mode": "fixed",
                    "type": "line",
                    "axis": axis,
                    "req": length,
                    "used_len": float(used.length_m),
                    "used_rad": "",
                    "auto_scaled": bool(abs(used.length_m - cmd.length_m) > 1e-6),
                    **m,
                }
            )

    # circle axis: z=xy plane circle, x= yz, y=xz per UI conventions
    for axis in ("z", "x", "y"):
        for rad in frange(0.05, circle_max, grid):
            cmd = TrajectoryCommand("circle", axis, radius_m=float(rad), length_m=0.05, rise_m=0.04, turns=1.0)
            dp, qt, fc, used = svc._best_track(robot, q_seed, start_position, cmd, steps)
            m = _metrics_from_track(robot, dp, qt, fc)
            rows.append(
                {
                    "mode": "fixed",
                    "type": "circle",
                    "axis": axis,
                    "req": rad,
                    "used_len": "",
                    "used_rad": float(used.radius_m),
                    "auto_scaled": bool(abs(used.radius_m - cmd.radius_m) > 1e-6),
                    **m,
                }
            )
    return rows


def run_prod_mode(config: dict, steps: int, line_max: float, circle_max: float, grid: float) -> list[dict]:
    from qt_app.viewmodels.simulation_viewmodel import SimulationViewModel

    vm = SimulationViewModel()
    rows: list[dict] = []
    for axis in ("x", "y", "z"):
        for length in frange(0.05, line_max, grid):
            cfg = copy.deepcopy(config)
            cfg["simulation_defaults"]["steps"] = int(steps)
            cmd = TrajectoryCommand("line", axis, radius_m=0.03, length_m=float(length), rise_m=0.02, turns=1.0)
            res = vm.run_trajectory_command(cfg, cmd)
            tr = res["cartesian_track"]
            used_l = float(tr["command"]["length_m"])
            req_l = float(cmd.length_m)
            rows.append(
                {
                    "mode": "prod",
                    "type": "line",
                    "axis": axis,
                    "req": length,
                    "used_len": used_l,
                    "used_rad": "",
                    "auto_scaled": bool(abs(used_l - req_l) > 1e-6),
                    "failures": float(tr["metrics"].get("ik_failure_count", 0)),
                    "max_err_m": float(tr["metrics"].get("max_position_error_m", 9)),
                    "mean_err_m": float(tr["metrics"].get("mean_position_error_m", 9)),
                    "max_joint_step_deg": float(tr["metrics"].get("max_joint_step_deg", 0)),
                    "failure_ratio": float(tr["metrics"].get("ik_failure_ratio", 1)),
                }
            )

    for axis in ("z", "x", "y"):
        for rad in frange(0.05, circle_max, grid):
            cfg = copy.deepcopy(config)
            cfg["simulation_defaults"]["steps"] = int(steps)
            cmd = TrajectoryCommand("circle", axis, radius_m=float(rad), length_m=0.05, rise_m=0.04, turns=1.0)
            res = vm.run_trajectory_command(cfg, cmd)
            tr = res["cartesian_track"]
            used_r = float(tr["command"]["radius_m"])
            req_r = float(cmd.radius_m)
            rows.append(
                {
                    "mode": "prod",
                    "type": "circle",
                    "axis": axis,
                    "req": rad,
                    "used_len": "",
                    "used_rad": used_r,
                    "auto_scaled": bool(abs(used_r - req_r) > 1e-6),
                    "failures": float(tr["metrics"].get("ik_failure_count", 0)),
                    "max_err_m": float(tr["metrics"].get("max_position_error_m", 9)),
                    "mean_err_m": float(tr["metrics"].get("mean_position_error_m", 9)),
                    "max_joint_step_deg": float(tr["metrics"].get("max_joint_step_deg", 0)),
                    "failure_ratio": float(tr["metrics"].get("ik_failure_ratio", 1)),
                }
            )
    return rows


def summarize(rows: list[dict]) -> None:
    sub = rows
    bad = [r for r in sub if r["auto_scaled"] or r["failures"] > 0 or r["max_err_m"] > 0.015]
    print(f"--- total={len(sub)} bad={len(bad)} (bad = scaled OR failures>0 OR max_err>15mm)")
    for kind in ("line", "circle"):
        for ax in ("x", "y", "z"):
            g = [r for r in sub if r["type"] == kind and r["axis"] == ax]
            if not g:
                continue
            ok = [x for x in g if not (x["auto_scaled"] or x["failures"] or x["max_err_m"] > 0.015)]
            print(
                f"  {kind:6} {ax}: tested={len(g)} ok(strict)={len(ok)} "
                f"worst_max_err={max(r['max_err_m'] for r in g):.4f} "
                f"worst_steps={max(r['max_joint_step_deg'] for r in g):.1f}deg "
                f"scaled_count={sum(1 for r in g if r['auto_scaled'])} "
                f"fail_count={sum(1 for r in g if r['failures']>0)}"
            )
            b2 = [r for r in bad if r["type"] == kind and r["axis"] == ax]
            for r in b2[:12]:
                print(
                    "    BAD",
                    r["mode"],
                    r["type"],
                    "axis=" + r["axis"],
                    "req=" + str(r["req"]),
                    "used_len=" + str(r.get("used_len")),
                    "used_rad=" + str(r.get("used_rad")),
                    "scaled=" + str(r["auto_scaled"]),
                    "fail=" + str(r["failures"]),
                    "max_err=" + str(round(r["max_err_m"], 5)),
                )


def main() -> int:
    ap = argparse.ArgumentParser(description="Sweep line/circle trajectories.")
    ap.add_argument("--mode", choices=("fixed", "prod"), default="fixed")
    ap.add_argument("--steps", type=int, default=31)
    ap.add_argument("--line-max", type=float, default=1.05)
    ap.add_argument("--circle-max", type=float, default=0.92)
    ap.add_argument("--grid", type=float, default=0.05)
    args = ap.parse_args()

    from python.robot_model import load_robot_config

    config = load_robot_config()
    rows = (
        run_prod_mode(config, args.steps, args.line_max, args.circle_max, args.grid)
        if args.mode == "prod"
        else run_fixed_mode(config, args.steps, args.line_max, args.circle_max, args.grid)
    )
    summarize(rows)
    robot = SixDofRobot(copy.deepcopy(config))
    est_r = TrajectoryService()._estimated_workspace_radius(robot)  # type: ignore[attr-defined]
    print(f"estimated_workspace_radius_m (heuristic): {est_r:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
