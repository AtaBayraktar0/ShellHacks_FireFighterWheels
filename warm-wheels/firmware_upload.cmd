@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Project Python is missing. Follow docs\windows-hardware.md.
  pause
  exit /b 1
)
echo Upload requires the inspected Uno COM port and explicit board confirmation.
echo Example: firmware_upload.cmd --port COM7 --confirm-port COM7 --confirm-uno
echo Use the actual COM port shown by hardware_preflight.cmd, not Bluetooth.
".venv\Scripts\python.exe" -m scripts.firmware upload %*
set "WW_EXIT=%ERRORLEVEL%"
echo.
pause
exit /b %WW_EXIT%
