@echo off
if "%~1"=="" (
  echo Usage: "%~nx0" path-to-session.json
  exit /b 2
)
call "%~dp0Open Propeller Arm.cmd" --matched-hbo "%~f1"
