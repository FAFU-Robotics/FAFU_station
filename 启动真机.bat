@echo off
setlocal EnableExtensions
cd /d "%~dp0"

REM Live arm: CPython 3.10 matches fafu_motor.cp310. ASCII-only so cmd.exe does not split UTF-8.

set "STATION_ALLOW_LIVE_ARM=1"

if exist "%~dp0app\station_desktop.py" (
  set "APP_DIR=%~dp0app"
  set "DESKTOP_PY=%~dp0app\station_desktop.py"
) else (
  set "APP_DIR=%~dp0"
  set "DESKTOP_PY=%~dp0station_desktop.py"
)

if exist "%APP_DIR%\vendor\fafu_arm_sdk\fafu_robot_python\fafu_robot_controller.py" (
  set "FAFU_ARM_SDK=%APP_DIR%\vendor\fafu_arm_sdk"
)
if not defined FAFU_ARM_SDK if exist "%~dp0vendor\fafu_arm_sdk\fafu_robot_python\fafu_robot_controller.py" (
  set "FAFU_ARM_SDK=%~dp0vendor\fafu_arm_sdk"
)

if exist "%~dp0runtime\python310\python.exe" if exist "%~dp0FAFUArmStation.exe" goto install_home
if exist "%~dp0runtime\python310\python.exe" goto portable

if not defined FAFU_ARM_SDK (
  if exist "D:\fafu_arm_sdk\fafu_robot_python\fafu_robot_controller.py" (
    set "FAFU_ARM_SDK=D:\fafu_arm_sdk"
  )
)
if not defined FAFU_ARM_SDK (
  if exist "%~dp0vendor\fafu_arm_sdk\fafu_robot_python\fafu_robot_controller.py" (
    set "FAFU_ARM_SDK=%~dp0vendor\fafu_arm_sdk"
  )
)

set "PY310=%LocalAppData%\Programs\Python\Python310\python.exe"
if not exist "%PY310%" (
  echo Python 3.10 not found.
  echo Official fafu_motor is cp310. Do not use Python 3.11 for live USB.
  echo To build a private runtime that does not change PATH:
  echo   powershell -File packaging\windows\build_portable.ps1
  echo Or install: winget install Python.Python.3.10
  pause
  exit /b 1
)

"%PY310%" -c "import websockets, yaml, numpy, pyrealsense2" 1>nul 2>nul
if errorlevel 1 (
  echo Installing station deps for Python 3.10 ...
  "%PY310%" -m pip install -q websockets PyYAML pywebview numpy pyrealsense2
  if errorlevel 1 (
    echo pip install failed
    pause
    exit /b 1
  )
)

echo Live station: Python 3.10  STATION_ALLOW_LIVE_ARM=1  arm=fafu  allow-motion
echo SDK=%FAFU_ARM_SDK%
echo E-stop: page button or unplug USB. Clear the workspace before Enable.
"%PY310%" -u "%DESKTOP_PY%" %*
if errorlevel 1 pause
exit /b %ERRORLEVEL%

:install_home
set "DEST=%LOCALAPPDATA%\FAFUArmStation"
set "HERE=%~dp0"
if /i "%HERE%"=="%DEST%\" goto portable
echo Installing private runtime to %DEST%
echo Does not change system PATH.
if not exist "%DEST%" mkdir "%DEST%"
robocopy "%~dp0." "%DEST%" /E /XD cache /NFL /NDL /NJH /NJS /nc /ns /np
if errorlevel 8 (
  echo Copy failed, starting from this folder.
  goto portable
)
if exist "%DEST%\install_shortcut.vbs" cscript //nologo "%DEST%\install_shortcut.vbs" "%DEST%"
if exist "%DEST%\FAFUArmStation.exe" (
  start "" "%DEST%\FAFUArmStation.exe"
  exit /b 0
)
goto portable

:portable
set "STATION_PORTABLE=1"
set "STATION_INSTALL_ROOT=%~dp0"
set "PYTHONNOUSERSITE=1"
set "PIP_USER=0"
set "PYTHONSTARTUP="
set "PYTHONHOME="
set "PYTHONUSERBASE="
set "PYTHONPATH=%APP_DIR%"
set "PATH=%~dp0runtime\python310;%~dp0runtime\python310\Scripts;%PATH%"
echo Live station: portable Python  STATION_ALLOW_LIVE_ARM=1  arm=fafu  allow-motion
echo SDK=%FAFU_ARM_SDK%
echo Private runtime. Does not change system PATH.
echo E-stop: page button or unplug USB. Clear the workspace before Enable.
"%~dp0runtime\python310\python.exe" -u "%DESKTOP_PY%" %*
if errorlevel 1 pause
exit /b %ERRORLEVEL%
