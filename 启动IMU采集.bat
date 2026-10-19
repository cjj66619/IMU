@echo off
cd /d "%~dp0"
if not exist "runtime\python\python.exe" (
  echo [ERROR] runtime\python\python.exe not found. Please copy the WHOLE IMU folder again.
  pause
  exit /b 1
)
title IMU Collector - close this window to quit
"runtime\python\python.exe" -X utf8 "imu_app\server.py"
if errorlevel 1 pause
