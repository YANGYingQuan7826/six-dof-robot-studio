from __future__ import annotations

from pathlib import Path
from typing import Any

from python.robot_model import SixDofRobot
from python.simulator import export_results, run_default_simulation, run_fk_demo


class SimulationService:
    """Coordinates simulation use cases for the ViewModel layer."""

    def run_default(self, config: dict[str, Any], output_root: str | Path | None = None) -> dict[str, Any]:
        return run_default_simulation(config, output_root=output_root)

    def run_fk(self, config: dict[str, Any]) -> dict[str, Any]:
        return run_fk_demo(config)

    def export_results_bundle(self, results: dict[str, Any], export_root: str | Path) -> None:
        export_root = Path(export_root)
        export_root.mkdir(parents=True, exist_ok=True)
        export_results(export_root / "qt_ptp", results["ptp"])
        export_results(export_root / "qt_cartesian_track", results["cartesian_track"])

    def robot_from_config(self, config: dict[str, Any]) -> SixDofRobot:
        return SixDofRobot(config)
