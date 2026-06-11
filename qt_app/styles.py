from __future__ import annotations

from PySide6.QtWidgets import QApplication


APP_STYLE = """
QMainWindow {
    background: #f5f5f7;
}

QFrame#LeftColumn {
    background: #f5f5f7;
    border-radius: 16px;
    border: 1px solid #e8e8ed;
}

QWidget#RightColumn {
    background: #ececee;
}

QFrame#AppCard {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #ffffff, stop:1 #f7f8ff);
    border: 1px solid #e0e7ff;
    border-radius: 18px;
}

QFrame#ToolStrip {
    background: rgba(17, 24, 39, 0.94);
    border: 1px solid #334155;
    border-radius: 18px;
}

QWidget#AdvancedPanel {
    background: #f8fafc;
    border: 1px solid #dbe4ff;
    border-radius: 14px;
}

QLabel#SectionTitle {
    font-size: 15px;
    font-weight: 700;
    color: #111827;
}

QLabel#TopBarTitle {
    font-size: 18px;
    font-weight: 700;
    color: #111827;
}

QLabel#TopBarSubtitle {
    color: #6b7280;
    font-size: 12px;
}

QLabel#StripTitle {
    font-size: 13px;
    font-weight: 700;
    color: #374151;
}

QLabel#LegendStrip {
    color: #4b5563;
    font-size: 11px;
}

QPushButton#GhostButton {
    background: transparent;
    border: 1px solid #d1d5db;
    border-radius: 8px;
    padding: 6px 12px;
    color: #374151;
}

QPushButton#GhostButton:hover {
    background: #f3f4f6;
}

QWidget {
    color: #1d1d1f;
    font-family: "Segoe UI", "Microsoft YaHei UI", "PingFang SC", sans-serif;
    font-size: 13px;
}

QLabel#TitleLabel {
    color: #1d1d1f;
    font-size: 28px;
    font-weight: 700;
}

QLabel#SubtitleLabel {
    color: #6e6e73;
    font-size: 13px;
}

QLabel#GuideLabel {
    color: #515154;
    font-size: 12px;
    line-height: 1.45;
}

QLabel#MetricCaption {
    color: #6e6e73;
    font-size: 12px;
}

QLabel#MetricValue {
    color: #1d1d1f;
    font-size: 16px;
    font-weight: 700;
}

QLabel#StatusPill {
    background: #e8f7ee;
    color: #16833a;
    border-radius: 12px;
    padding: 6px 12px;
    font-weight: 700;
}

QGroupBox {
    background: #ffffff;
    border: 1px solid #e5e5ea;
    border-radius: 18px;
    margin-top: 18px;
    padding: 18px 16px 16px 16px;
    font-size: 14px;
    font-weight: 650;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 16px;
    padding: 0 6px;
    color: #1d1d1f;
}

QLineEdit, QPlainTextEdit, QTableWidget, QComboBox {
    background: #fbfbfd;
    border: 1px solid #d2d2d7;
    border-radius: 10px;
    selection-background-color: #007aff;
    selection-color: white;
}

QLineEdit, QComboBox {
    min-height: 32px;
    padding: 2px 10px;
}

QLineEdit#CommandInput {
    min-height: 42px;
    border: 1px solid #b6c7ff;
    border-radius: 16px;
    padding: 4px 16px;
    font-size: 14px;
    background: #ffffff;
}

QComboBox::drop-down {
    border: 0;
    width: 24px;
}

QPlainTextEdit {
    padding: 10px;
    line-height: 1.4;
}

QTableWidget {
    gridline-color: #e5e5ea;
    alternate-background-color: #f8f8fb;
}

QHeaderView::section {
    background: #f2f2f7;
    color: #424245;
    border: 0;
    border-bottom: 1px solid #d2d2d7;
    padding: 8px;
    font-weight: 600;
}

QPushButton {
    background: #ffffff;
    border: 1px solid #d2d2d7;
    border-radius: 10px;
    padding: 9px 14px;
    color: #1d1d1f;
    font-weight: 600;
}

QPushButton:hover {
    background: #f5f5f7;
}

QPushButton:pressed {
    background: #e8e8ed;
}

QPushButton#PrimaryButton {
    background: #007aff;
    border: 1px solid #007aff;
    color: white;
}

QPushButton#PrimaryButton:hover {
    background: #0a84ff;
}

QPushButton#PrimaryButton:pressed {
    background: #0066d6;
}

QPushButton#PlayButton {
    min-width: 48px;
    min-height: 42px;
    border-radius: 21px;
    border: 1px solid #60a5fa;
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #38bdf8, stop:1 #2563eb);
    color: white;
    font-size: 18px;
    font-weight: 900;
    padding: 0;
}

QPushButton#PlayButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #7dd3fc, stop:1 #3b82f6);
}

QPushButton#PlayButton:pressed {
    background: #1d4ed8;
}

QPushButton#IconButton {
    min-width: 42px;
    min-height: 42px;
    border-radius: 21px;
    border: 1px solid #475569;
    background: #111827;
    color: #e5e7eb;
    font-size: 17px;
    font-weight: 800;
    padding: 0;
}

QPushButton#IconButton:hover {
    background: #1f2937;
    border-color: #60a5fa;
    color: #ffffff;
}

QPushButton#IconButton:pressed {
    background: #020617;
}

QFrame#MetricCard {
    background: #fbfbfd;
    border: 1px solid #e5e5ea;
    border-radius: 14px;
}

QSlider::groove:horizontal {
    height: 6px;
    background: #e5e5ea;
    border-radius: 3px;
}

QSlider::handle:horizontal {
    width: 18px;
    height: 18px;
    margin: -6px 0;
    border-radius: 9px;
    background: #007aff;
}

QSlider::sub-page:horizontal {
    background: #007aff;
    border-radius: 3px;
}

QSplitter::handle {
    background: transparent;
}

QStatusBar {
    background: #f5f5f7;
    color: #6e6e73;
}
"""


def apply_app_style(app: QApplication) -> None:
    app.setStyleSheet(APP_STYLE)
