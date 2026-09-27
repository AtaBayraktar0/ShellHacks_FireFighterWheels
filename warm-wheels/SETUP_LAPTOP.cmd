@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" python -m venv .venv
if errorlevel 1 goto :failed
".venv\Scripts\python.exe" -m pip install -r requirements.txt -r requirements-dev.txt pyrealsense2
if errorlevel 1 goto :failed
echo Base laptop environment ready. START_PRESENTATION.cmd runs without any hardware.
echo AI uses the separate .venv-ai environment; see docs\presentation-guide.md.
pause
exit /b 0
:failed
echo Setup failed. Keep this window and its error for troubleshooting.
pause
exit /b 1
