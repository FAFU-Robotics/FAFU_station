@echo off
setlocal EnableExtensions
set "PY310=C:\Users\AAA\AppData\Local\Programs\Python\Python310"
set "CMAKE=C:\Program Files\CMake\bin"
set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
set "SDK_CPP=D:\fafu_arm_sdk\fafu_robot_cpp"

if not exist "%PY310%\python.exe" (
  echo Python 3.10 not found at %PY310%
  exit /b 1
)
if not exist "%VCVARS%" (
  echo VS2022 vcvars64.bat not found
  exit /b 1
)
if not exist "%CMAKE%\cmake.exe" (
  echo cmake.exe not found in %CMAKE%
  exit /b 1
)

call "%VCVARS%"
if errorlevel 1 exit /b 1
set "PATH=%PY310%;%PY310%\Scripts;%CMAKE%;%PATH%"
cd /d "%SDK_CPP%"
call build.bat
exit /b %ERRORLEVEL%
