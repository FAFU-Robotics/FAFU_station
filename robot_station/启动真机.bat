@echo off
setlocal EnableExtensions
cd /d "%~dp0"

REM Live arm: select a Python that can import this SDK. Keep this file ASCII-only.

set "STATION_ALLOW_LIVE_ARM=1"

if exist "%~dp0app\station_desktop.py" (
  set "APP_DIR=%~dp0app"
  set "DESKTOP_PY=%~dp0app\station_desktop.py"
) else (
  set "APP_DIR=%~dp0"
  set "DESKTOP_PY=%~dp0station_desktop.py"
)

if exist "%APP_DIR%\urdf" (
  set "STATION_URDF_DIR=%APP_DIR%\urdf"
) else if exist "%~dp0urdf" (
  set "STATION_URDF_DIR=%~dp0urdf"
)

if exist "%APP_DIR%\vendor\fafu_arm_sdk\fafu_robot_python\fafu_robot_controller.py" (
  set "FAFU_ARM_SDK=%APP_DIR%\vendor\fafu_arm_sdk"
)
if not defined FAFU_ARM_SDK if exist "%~dp0vendor\fafu_arm_sdk\fafu_robot_python\fafu_robot_controller.py" (
  set "FAFU_ARM_SDK=%~dp0vendor\fafu_arm_sdk"
)

REM Portable package: run in place. Do not robocopy the tree before the window.
if exist "%~dp0runtime\python310\python.exe" goto portable

if not defined FAFU_ARM_SDK if exist "%~dp0..\fafu_arm_sdk\fafu_robot_python\fafu_robot_controller.py" (
  set "FAFU_ARM_SDK=%~dp0..\fafu_arm_sdk"
)

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

REM Source tree: prefer Python 3.12, retaining 3.10 if its extension is available.
set "STATION_PY="
call :pick_python
if not defined STATION_PY (
  echo No Python could import the selected SDK: %FAFU_ARM_SDK%
  echo Build its extension with scripts\rebuild_fafu_motor.bat first.
  echo Set STATION_PYTHON to choose a specific python.exe.
  if /i "%~1"=="--check-python" exit /b 1
  pause
  exit /b 1
)
if /i "%~1"=="--check-python" (
  echo %STATION_PY%
  exit /b 0
)

for %%D in ("%STATION_PY%") do set "PYDIR=%%~dpD"
if exist "%PYDIR%Library\bin" set "PATH=%PYDIR%;%PYDIR%Scripts;%PYDIR%Library\bin;%PATH%"

"%STATION_PY%" -c "import websockets, yaml, numpy, pyrealsense2, webview" 1>nul 2>nul
if errorlevel 1 (
  echo Installing station dependencies ...
  "%STATION_PY%" -m pip install -q websockets PyYAML pywebview numpy pyrealsense2
  if errorlevel 1 (
    echo pip install failed
    pause
    exit /b 1
  )
)

set "PIN=NO"
"%STATION_PY%" -c "import pinocchio" 1>nul 2>nul
if not errorlevel 1 set "PIN=YES"

echo Live station: STATION_ALLOW_LIVE_ARM=1  arm=fafu  allow-motion
echo PY=%STATION_PY%
echo pinocchio=%PIN%
echo SDK=%FAFU_ARM_SDK%
echo E-stop: page button or unplug USB. Clear the workspace before Enable.
"%STATION_PY%" -u "%DESKTOP_PY%" %*
if errorlevel 1 pause
exit /b %ERRORLEVEL%

:portable
set "STATION_PORTABLE=1"
set "STATION_INSTALL_ROOT=%~dp0"
set "PYTHONNOUSERSITE=1"
set "PIP_USER=0"
set "PYTHONSTARTUP="
set "PYTHONHOME="
set "PYTHONUSERBASE="
set "PYTHONPATH=%APP_DIR%"
if exist "%~dp0runtime\python310\Library\bin" (
  set "PINOCCHIO_WINDOWS_DLL_PATH=%~dp0runtime\python310\Library\bin"
  set "PATH=%~dp0runtime\python310\Library\bin;%~dp0runtime\python310;%~dp0runtime\python310\Scripts;%PATH%"
) else (
  set "PATH=%~dp0runtime\python310;%~dp0runtime\python310\Scripts;%PATH%"
)
if exist "%~dp0install_shortcut.vbs" (
  start "" /b cscript //nologo "%~dp0install_shortcut.vbs" "%~dp0"
)
echo Live station: portable Python  STATION_ALLOW_LIVE_ARM=1  arm=fafu  allow-motion
echo SDK=%FAFU_ARM_SDK%
echo Private runtime in place. Does not change system PATH.
echo E-stop: page button or unplug USB. Clear the workspace before Enable.
"%~dp0runtime\python310\python.exe" -u "%DESKTOP_PY%" %*
if errorlevel 1 pause
exit /b %ERRORLEVEL%

:pick_python
if defined STATION_PYTHON (
  call :try_python "%STATION_PYTHON%"
  goto :eof
)
if defined VIRTUAL_ENV call :try_python "%VIRTUAL_ENV%\Scripts\python.exe"
if defined CONDA_PREFIX call :try_python "%CONDA_PREFIX%\python.exe"
if defined STATION_PY goto :eof
for /f "delims=" %%P in ('py -3.12 -c "import sys; print(sys.executable)" 2^>nul') do call :try_python "%%P"
if defined STATION_PY goto :eof
for /f "delims=" %%P in ('py -3.10 -c "import sys; print(sys.executable)" 2^>nul') do call :try_python "%%P"
if defined STATION_PY goto :eof
for /f "delims=" %%P in ('python -c "import sys; print(sys.executable)" 2^>nul') do call :try_python "%%P"
if defined STATION_PY goto :eof
call :try_python "%LocalAppData%\Programs\Python\Python312\python.exe"
call :try_python "%LocalAppData%\Programs\Python\Python310\python.exe"
goto :eof

:try_python
if defined STATION_PY goto :eof
if not exist "%~1" goto :eof
"%~1" -c "import sys; from pathlib import Path; sys.path.insert(0, str(Path(sys.argv[1]) / 'fafu_robot_python')); import fafu_motor; import fafu_robot_controller" "%FAFU_ARM_SDK%" 1>nul 2>nul
if not errorlevel 1 set "STATION_PY=%~1"
goto :eof
