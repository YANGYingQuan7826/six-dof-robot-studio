@echo off
cd /d "%~dp0"
echo Starting 6-DOF Robot Studio...
python -m qt_app.main
if errorlevel 1 (
    echo.
    echo Application failed to start. Please copy the error message above.
    pause
)
