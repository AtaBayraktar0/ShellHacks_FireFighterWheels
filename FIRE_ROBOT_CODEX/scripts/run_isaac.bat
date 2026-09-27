@echo off
rem Created by OpenAI Codex: launch with the desktop's Isaac Sim 6.1 interpreter.
setlocal
if not defined ISAAC_SIM_PATH (
  echo Set ISAAC_SIM_PATH to your Isaac Sim 6.1 installation directory.
  exit /b 1
)
call "%ISAAC_SIM_PATH%\python.bat" "%~dp0..\run_isaac.py" %*
exit /b %errorlevel%
