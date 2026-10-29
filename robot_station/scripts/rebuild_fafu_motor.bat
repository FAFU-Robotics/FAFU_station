@echo off
setlocal EnableExtensions
REM Usage: rebuild_fafu_motor.bat [Release|Debug]
REM STATION_PYTHON selects the target interpreter; otherwise use Python on PATH.
set "SDK_ROOT=%FAFU_ARM_SDK%"
if not defined SDK_ROOT set "SDK_ROOT=%~dp0..\..\fafu_arm_sdk"
if not exist "%SDK_ROOT%\fafu_robot_cpp\build.bat" (
    echo SDK not found: %SDK_ROOT%
    echo Run git submodule update --init --recursive from the repository root.
    exit /b 1
)
set "BUILD_TYPE=%~1"
if not defined BUILD_TYPE set "BUILD_TYPE=Release"
set "PY_EXE=%STATION_PYTHON%"
if not defined PY_EXE set "PY_EXE=python"
call "%SDK_ROOT%\fafu_robot_cpp\build.bat" "%BUILD_TYPE%" "%PY_EXE%"
exit /b %ERRORLEVEL%
