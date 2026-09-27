@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv-ai\Scripts\python.exe" (
  echo AI Python is missing. Follow docs\model-provenance.md.
  pause
  exit /b 1
)
".venv-ai\Scripts\python.exe" -m scripts.start_ai %*
set "WW_EXIT=%ERRORLEVEL%"
echo.
pause
exit /b %WW_EXIT%
