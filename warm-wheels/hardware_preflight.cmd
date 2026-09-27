@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Project Python is missing. Follow docs\windows-hardware.md.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -m scripts.hardware_preflight %*
set "WW_EXIT=%ERRORLEVEL%"
echo.
pause
exit /b %WW_EXIT%
