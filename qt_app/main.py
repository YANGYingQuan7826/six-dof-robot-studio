from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication


def main() -> int:
    workspace_root = Path(__file__).resolve().parent.parent
    if str(workspace_root) not in sys.path:
        sys.path.insert(0, str(workspace_root))

    from qt_app.viewmodels.simulation_viewmodel import SimulationViewModel
    from qt_app.styles import apply_app_style
    from qt_app.views.main_window import MainWindow

    app = QApplication(sys.argv)
    apply_app_style(app)
    window = MainWindow(SimulationViewModel())
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
