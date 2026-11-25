@echo off
if not exist "%~dp0.venv\Scripts\pythonw.exe" (
  echo Create the Python environment first. See README.md.
  pause
  exit /b 1
)
start "" "%~dp0.venv\Scripts\pythonw.exe" "%~dp0dashboard.py" %*
