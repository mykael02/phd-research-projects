@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Run setup.ps1 first to install the local environment.
  pause
  exit /b 1
)
echo MuJoCo balancing robot - starts PAUSED
echo SPACE = run/pause, J = push -X, L = push +X, R = reset.
echo Close the viewer to exit. This is an unvalidated starter model.
".venv\Scripts\python.exe" run.py --viewer
if errorlevel 1 pause
