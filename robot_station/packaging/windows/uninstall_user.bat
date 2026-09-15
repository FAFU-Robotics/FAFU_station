@echo off
setlocal EnableExtensions
set "DEST=%LOCALAPPDATA%\FAFUArmStation"
set "LNK=%USERPROFILE%\Desktop\FAFUArmStation.lnk"

echo Uninstall private runtime:
echo   %DEST%
echo Shortcut:
echo   %LNK%
echo.
echo This does not change system PATH or other Python installs.
echo Close FAFUArmStation before continuing.
pause

cd /d "%TEMP%"
if exist "%LNK%" del /f /q "%LNK%" >nul 2>&1
if exist "%DEST%" rmdir /s /q "%DEST%"
if exist "%DEST%" (
  echo Could not delete %DEST% - close the app and retry.
  pause
  exit /b 1
)
echo Removed.
pause
exit /b 0
