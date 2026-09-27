@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Run SETUP_LAPTOP.cmd first.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -m scripts.launch --mode demo %*
if errorlevel 1 pause
