# Build a private-runtime folder for customers.
# Does not install Python globally, does not write the user/system PATH.
#
# Output: <repo>\dist\FAFUArmStation\
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File packaging\windows\build_portable.ps1
#   powershell -File packaging\windows\build_portable.ps1 -Force
#   powershell -File packaging\windows\build_portable.ps1 -SkipSdk

[CmdletBinding()]
param(
    [string]$OutDir = "",
    [string]$PythonVersion = "3.10.11",
    [switch]$Force,
    [switch]$SkipSdk
)

$ErrorActionPreference = "Stop"
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$PackDir = $PSScriptRoot
$Repo = (Resolve-Path (Join-Path $PackDir "..\..")).Path
if (-not $OutDir) {
    $OutDir = Join-Path $Repo "dist\FAFUArmStation"
}
$Cache = Join-Path $PackDir "cache"
$PyZipName = "python-$PythonVersion-embed-amd64.zip"
$PyUrl = "https://www.python.org/ftp/python/$PythonVersion/$PyZipName"
$GetPipUrls = @(
    "https://bootstrap.pypa.io/get-pip.py",
    "https://github.com/pypa/get-pip/raw/main/public/get-pip.py"
)

function Get-Csc {
    $candidates = @(
        "$env:WINDIR\Microsoft.NET\Framework64\v4.0.30319\csc.exe",
        "$env:WINDIR\Microsoft.NET\Framework\v4.0.30319\csc.exe"
    )
    foreach ($p in $candidates) {
        if (Test-Path $p) { return $p }
    }
    throw "csc.exe not found. Windows includes .NET Framework 4."
}

function Find-Sdk {
    $envSdk = [string]$env:FAFU_ARM_SDK
    $candidates = @()
    if ($envSdk) { $candidates += $envSdk }
    $candidates += @(
        "D:\fafu_arm_sdk",
        (Join-Path $Repo "vendor\fafu_arm_sdk"),
        (Join-Path $Repo "..\fafu_arm_sdk"),
        (Join-Path $Repo "..\fafu_arm_sdk-main"),
        (Join-Path $env:USERPROFILE "fafu_arm_sdk"),
        (Join-Path $env:USERPROFILE "fafu_arm_sdk-main")
    )
    foreach ($c in $candidates) {
        $ctrl = Join-Path $c "fafu_robot_python\fafu_robot_controller.py"
        if (Test-Path $ctrl) { return (Resolve-Path $c).Path }
    }
    return $null
}

function Copy-Tree($src, $dst, [string[]]$ExtraXd = @()) {
    if (-not (Test-Path $src)) { throw "missing $src" }
    New-Item -ItemType Directory -Force -Path $dst | Out-Null
    $xd = @("__pycache__", ".git", ".venv", "venv", "dist", "build") + @($ExtraXd)
    $xf = @("*.pyc", "*.pyo")
    $args = @($src, $dst, "/E", "/NFL", "/NDL", "/NJH", "/NJS", "/nc", "/ns", "/np", "/XD") + $xd + @("/XF") + $xf
    & robocopy @args | Out-Null
    if ($LASTEXITCODE -ge 8) { throw "robocopy failed $src -> $dst (code $LASTEXITCODE)" }
}

Write-Host "Repo:    $Repo"
Write-Host "Output:  $OutDir"

if ($Force -and (Test-Path $OutDir)) {
    Remove-Item -Recurse -Force $OutDir
}
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
New-Item -ItemType Directory -Force -Path $Cache | Out-Null

$runtime = Join-Path $OutDir "runtime\python310"
$app = Join-Path $OutDir "app"
New-Item -ItemType Directory -Force -Path $runtime | Out-Null
New-Item -ItemType Directory -Force -Path $app | Out-Null

$zip = Join-Path $Cache $PyZipName
if ($Force -or -not (Test-Path $zip)) {
    Write-Host "Downloading $PyUrl"
    Invoke-WebRequest -Uri $PyUrl -OutFile $zip
}

$pyExe = Join-Path $runtime "python.exe"
if ($Force -or -not (Test-Path $pyExe)) {
    Write-Host "Extracting embeddable Python $PythonVersion"
    if (Test-Path $runtime) {
        Get-ChildItem $runtime | Remove-Item -Recurse -Force
    }
    New-Item -ItemType Directory -Force -Path $runtime | Out-Null
    Expand-Archive -LiteralPath $zip -DestinationPath $runtime -Force
}

$pthSrc = Join-Path $PackDir "python310._pth"
Copy-Item -Force $pthSrc (Join-Path $runtime "python310._pth")

$getPip = Join-Path $Cache "get-pip.py"
$needPip = $Force -or -not (Test-Path $getPip) -or ((Get-Item $getPip).Length -lt 10000)
if ($needPip) {
    Write-Host "Downloading get-pip.py"
    $ok = $false
    foreach ($url in $GetPipUrls) {
        try {
            Invoke-WebRequest -Uri $url -OutFile $getPip
            if ((Test-Path $getPip) -and ((Get-Item $getPip).Length -gt 10000)) {
                $ok = $true
                break
            }
        } catch {
            Write-Warning "get-pip download failed from $url"
        }
    }
    if (-not $ok) { throw "Could not download get-pip.py" }
}

$envKeys = @(
    "PYTHONNOUSERSITE", "PIP_USER", "PYTHONSTARTUP", "PYTHONPATH",
    "STATION_PORTABLE", "PYTHONHOME", "PYTHONUSERBASE"
)
$envBackup = @{}
foreach ($k in $envKeys) {
    $envBackup[$k] = [Environment]::GetEnvironmentVariable($k, "Process")
}

try {
    $env:PYTHONNOUSERSITE = "1"
    $env:PIP_USER = "0"
    $env:PYTHONSTARTUP = ""
    $env:PYTHONHOME = ""
    $env:PYTHONUSERBASE = ""
    $env:PYTHONPATH = $app
    $env:STATION_PORTABLE = "1"

    Write-Host "Installing pip into private runtime"
    & $pyExe $getPip --no-warn-script-location --no-user
    if ($LASTEXITCODE -ne 0) { throw "get-pip failed" }

    # pip may rewrite python310._pth — restore isolation + app path.
    Copy-Item -Force $pthSrc (Join-Path $runtime "python310._pth")

    $req = Join-Path $Repo "requirements.txt"
    Write-Host "Installing station dependencies into private runtime"
    & $pyExe -m pip install --no-warn-script-location --no-user -r $req numpy
    if ($LASTEXITCODE -ne 0) { throw "pip install failed" }
    Copy-Item -Force $pthSrc (Join-Path $runtime "python310._pth")
}
finally {
    foreach ($k in $envKeys) {
        $v = $envBackup[$k]
        if ([string]::IsNullOrEmpty($v)) {
            Remove-Item "Env:$k" -ErrorAction SilentlyContinue
        } else {
            Set-Item -Path "Env:$k" -Value $v
        }
    }
}

Write-Host "Copying app"
Copy-Tree (Join-Path $Repo "robot_station") (Join-Path $app "robot_station")
Copy-Item -Force (Join-Path $Repo "run_station.py") $app
Copy-Item -Force (Join-Path $Repo "station_desktop.py") $app
Copy-Item -Force (Join-Path $Repo "station_client.conf") $app
Copy-Item -Force (Join-Path $Repo "requirements.txt") $app
Copy-Tree (Join-Path $Repo "configs") (Join-Path $app "configs")
$packagedYaml = Join-Path $app "configs\station.yaml"
if (Test-Path $packagedYaml) {
    Remove-Item -Force $packagedYaml
}
$docsApp = Join-Path $app "docs"
New-Item -ItemType Directory -Force -Path $docsApp | Out-Null
foreach ($doc in @("使用说明.md", "SPEC.md")) {
    $p = Join-Path $Repo "docs\$doc"
    if (Test-Path $p) { Copy-Item -Force $p $docsApp }
}
Copy-Item -Force (Join-Path $Repo "README.md") $app

$sdkDest = Join-Path $app "vendor\fafu_arm_sdk"
$sdk = $null
if (-not $SkipSdk) { $sdk = Find-Sdk }
if ($sdk) {
    Write-Host "Copying SDK $sdk"
    New-Item -ItemType Directory -Force -Path (Join-Path $app "vendor") | Out-Null
    Copy-Tree $sdk $sdkDest @(".github")
} else {
    Write-Warning "fafu_arm_sdk not found. Package will run simulation; live USB needs the SDK in app\vendor\fafu_arm_sdk"
    New-Item -ItemType Directory -Force -Path (Join-Path $app "vendor") | Out-Null
    @(
        "Place official fafu_arm_sdk here (must contain fafu_robot_python).",
        "Live USB will not work until this folder is present and the package is rebuilt."
    ) | Set-Content -Encoding ascii (Join-Path $app "vendor\PUT_SDK_HERE.txt")
}

Write-Host "Compiling FAFUArmStation.exe"
$csc = Get-Csc
$exe = Join-Path $OutDir "FAFUArmStation.exe"
$cs = Join-Path $PackDir "PortableLauncher.cs"
& $csc /nologo /target:winexe /platform:anycpu /utf8output /out:$exe /r:System.dll /r:System.Windows.Forms.dll /r:System.Drawing.dll $cs
if ($LASTEXITCODE -ne 0) { throw "csc failed" }

Copy-Item -Force (Join-Path $PackDir "install_shortcut.vbs") $OutDir
Copy-Item -Force (Join-Path $PackDir "install_user.bat") (Join-Path $OutDir "Install.bat")
Copy-Item -Force (Join-Path $PackDir "uninstall_user.bat") (Join-Path $OutDir "Uninstall.bat")
Copy-Item -Force (Join-Path $Repo "start_live_arm.bat") (Join-Path $OutDir "StartLive.bat")
Copy-Item -Force (Join-Path $Repo "启动真机.bat") (Join-Path $OutDir "启动真机.bat")

$readme = @"
FAFU Arm Station — private runtime
===================================

This folder is self-contained. It does NOT install Python for Windows,
does not add Python to PATH, and does not change other Python environments.

First run
- Plug the arm USB into THIS PC
- Double-click FAFUArmStation.exe (or 启动真机.bat)
- First launch copies to %LOCALAPPDATA%\FAFUArmStation, creates a desktop shortcut, and opens the page
- After USB connects, the app Connects and enables the arm (motors hold pose)

Later
- Double-click the desktop FAFUArmStation shortcut

Uninstall: Uninstall.bat in %LOCALAPPDATA%\FAFUArmStation, or delete that folder and the shortcut.

USB
- Official fafu_arm_sdk must be inside app\vendor\fafu_arm_sdk
- Windows must see a COM port (USB serial driver is system-wide; this package cannot isolate kernel drivers)

Simulation
- Open the app and choose the sim arm on the right
"@
$readme | Set-Content -Encoding utf8 (Join-Path $OutDir "Readme.txt")

Write-Host ""
Write-Host "OK: $OutDir"
Write-Host "Launcher: $exe"
Write-Host "Python:   $pyExe"
if ($sdk) { Write-Host "SDK:      $sdkDest" } else { Write-Host "SDK:      (missing)" }
Write-Host "Next: plug USB, then double-click FAFUArmStation.exe  (does not change PATH)"
