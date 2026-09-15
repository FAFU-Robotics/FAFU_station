@echo off
setlocal EnableExtensions
cd /d "%~dp0"
chcp 65001 >nul

set "DEST=%LOCALAPPDATA%\FAFUArmStation"
echo Installing private runtime to:
echo   %DEST%
echo.
echo This copy is only for this app.
echo It does NOT add Python to PATH, does NOT use setx,
echo and does NOT change the system's Python.
echo.

if not exist "%~dp0runtime\python310\python.exe" (
  echo ERROR: runtime\python310\python.exe missing.
  echo Build first: powershell -File packaging\windows\build_portable.ps1
  pause
  exit /b 1
)
if not exist "%~dp0FAFUArmStation.exe" (
  echo ERROR: FAFUArmStation.exe missing
  pause
  exit /b 1
)

if not exist "%DEST%" mkdir "%DEST%"
robocopy "%~dp0." "%DEST%" /E /XD cache /NFL /NDL /NJH /NJS /nc /ns /np
if errorlevel 8 (
  echo Copy failed
  pause
  exit /b 1
)

cscript //nologo "%DEST%\install_shortcut.vbs" "%DEST%"
echo.
start "" "%DEST%\FAFUArmStation.exe"
exit /b 0
