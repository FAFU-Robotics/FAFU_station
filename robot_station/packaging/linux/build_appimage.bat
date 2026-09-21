@echo off
setlocal EnableExtensions
REM Optional Windows entry: Docker Desktop linux/amd64.
REM If docker is missing, fail with a readable message (use GitHub Actions or Ubuntu 22.04).

cd /d "%~dp0"
set "PACK=%~dp0"
for %%I in ("%PACK%..\..") do set "STATION=%%~fI"
for %%I in ("%STATION%\..") do set "REPO=%%~fI"

where docker >nul 2>&1
if errorlevel 1 (
  echo Docker Desktop is not available on this Windows machine.
  echo AppImage must be built on Ubuntu 22.04 x86_64.
  echo Use GitHub Actions workflow_dispatch on packaging-smoke.yml,
  echo or run bash packaging/linux/build_appimage.sh on Ubuntu 22.04.
  echo Do not build the customer AppImage on Windows Python / WSL USB tests.
  exit /b 1
)

if not exist "%REPO%\robot_station\packaging\linux\Dockerfile" (
  echo Expected Dockerfile at %REPO%\robot_station\packaging\linux\Dockerfile
  echo Run this bat from a FAFU_station checkout that contains robot_station.
  exit /b 1
)

echo Building image fafu-appimage from %REPO%
docker build -f "%REPO%\robot_station\packaging\linux\Dockerfile" -t fafu-appimage "%REPO%"
if errorlevel 1 (
  echo docker build failed
  exit /b 1
)

echo Running packer. Log: robot_station\dist\appimage-build.log
docker run --rm -e APPIMAGE_EXTRACT_AND_RUN=1 -v "%REPO%:/src" fafu-appimage %*
if errorlevel 1 (
  echo docker run failed. See robot_station\dist\appimage-build.log
  exit /b 1
)

echo.
echo Send: %STATION%\dist\FAFUArmStation-x86_64.AppImage
echo Manifest: %STATION%\dist\BUILD_MANIFEST.json
exit /b 0
