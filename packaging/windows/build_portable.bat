@echo off
setlocal EnableExtensions
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_portable.ps1" %*
if errorlevel 1 (
  echo Build failed
  pause
  exit /b 1
)
echo.
echo Output: %~dp0..\..\dist\FAFUArmStation
pause
exit /b 0
