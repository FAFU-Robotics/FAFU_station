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
echo Folder: %~dp0..\..\dist\FAFUArmStation
echo Send:   %~dp0..\..\dist\FAFUArmStation.exe
pause
exit /b 0
