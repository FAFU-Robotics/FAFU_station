# Build a private-runtime folder for customers.
# Does not install Python globally, does not write the user/system PATH.
#
# Output: <repo>\dist\FAFUArmStation\  (portable tree)
#         <repo>\dist\FAFUArmStation.exe  (send this one file; skipped with -SkipSingleExe)
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File packaging\windows\build_portable.ps1
#   powershell -File packaging\windows\build_portable.ps1 -Force
#   powershell -File packaging\windows\build_portable.ps1 -SkipSdk
#   powershell -File packaging\windows\build_portable.ps1 -WithPinocchio
#   powershell -File packaging\windows\build_portable.ps1 -SkipSingleExe

[CmdletBinding()]
param(
    [string]$OutDir = "",
    [string]$PythonVersion = "3.10.11",
    [switch]$Force,
    [switch]$SkipSdk,
    [switch]$SkipPinocchio,
    [switch]$WithPinocchio,
    [switch]$SkipSingleExe
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

function Test-PinocchioPython([string]$PyExe) {
    if (-not $PyExe -or -not (Test-Path $PyExe)) { return $false }
    $prev = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        & $PyExe -c "import sys,pinocchio; assert sys.version_info[:2]==(3,10)" 1>$null 2>$null
        return ($LASTEXITCODE -eq 0)
    } catch {
        return $false
    } finally {
        $ErrorActionPreference = $prev
    }
}

function Find-PinocchioEnv {
    $candidates = @()
    if ($env:STATION_PINOCCHIO) { $candidates += $env:STATION_PINOCCHIO }
    if ($env:STATION_PYTHON) { $candidates += $env:STATION_PYTHON }
    if ($env:CONDA_PREFIX) { $candidates += (Join-Path $env:CONDA_PREFIX "python.exe") }
    $candidates += @(
        "D:\Anaconda\envs\panthera\python.exe",
        "D:\Anaconda\envs\fafu-station\python.exe",
        (Join-Path $env:LOCALAPPDATA "anaconda3\envs\panthera\python.exe"),
        (Join-Path $env:LOCALAPPDATA "anaconda3\envs\fafu-station\python.exe"),
        (Join-Path $env:USERPROFILE "miniconda3\envs\fafu-station\python.exe")
    )
    foreach ($c in $candidates) {
        $exe = [string]$c
        if (-not $exe) { continue }
        if ((Test-Path $exe) -and (Test-Path $exe -PathType Container)) {
            $exe = Join-Path $exe "python.exe"
        }
        if (Test-PinocchioPython $exe) { return (Resolve-Path $exe).Path }
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

$pinocchioPy = $null
$wantPinocchio = $WithPinocchio -and -not $SkipPinocchio
if ($wantPinocchio) {
    $pinocchioPy = Find-PinocchioEnv
    if (-not $pinocchioPy) {
        throw "pinocchio (Python 3.10) not found. Windows pip cannot install the real library. Use a conda env: conda create -n fafu-station -c conda-forge python=3.10 pinocchio  then rebuild, or set STATION_PYTHON / STATION_PINOCCHIO. Default builds skip pinocchio and use station G(q) + SDK apply_compensation_torque."
    }
    $bundlePy = Join-Path $PackDir "bundle_pinocchio.py"
    Write-Host "Bundling pinocchio from $pinocchioPy"
    & $pinocchioPy $bundlePy --from $pinocchioPy --to $runtime
    if ($LASTEXITCODE -ne 0) { throw "bundle_pinocchio.py failed" }
    $dllDir = Join-Path $runtime "Library\bin"
    $env:PINOCCHIO_WINDOWS_DLL_PATH = $dllDir
    $env:PATH = "$dllDir;$runtime;$env:PATH"
    Copy-Item -Force $pthSrc (Join-Path $runtime "python310._pth")
    $prevEA = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        & $pyExe -c "import pinocchio; print('portable pinocchio', pinocchio.__version__)"
        if ($LASTEXITCODE -ne 0) { throw "portable runtime cannot import pinocchio" }
    } finally {
        $ErrorActionPreference = $prevEA
    }
} else {
    Write-Host "Skipping pinocchio (default). Gravity uses station G(q) + SDK apply_compensation_torque. Pass -WithPinocchio to bundle conda pinocchio."
}

Write-Host "Copying app"
Copy-Tree (Join-Path $Repo "robot_station") (Join-Path $app "robot_station")
Copy-Tree (Join-Path $Repo "urdf") (Join-Path $app "urdf")
Copy-Item -Force (Join-Path $Repo "run_station.py") $app
Copy-Item -Force (Join-Path $Repo "station_desktop.py") $app
Copy-Item -Force (Join-Path $Repo "station_client.conf") $app
Copy-Item -Force (Join-Path $Repo "requirements.txt") $app
Copy-Tree (Join-Path $Repo "configs") (Join-Path $app "configs")
$packagedYaml = Join-Path $app "configs\station.yaml"
if (Test-Path $packagedYaml) {
    Remove-Item -Force $packagedYaml
}
$docsSrc = Join-Path $Repo "docs"
$docsApp = Join-Path $app "docs"
New-Item -ItemType Directory -Force -Path $docsApp | Out-Null
$customerReadme = $null
Get-ChildItem -LiteralPath $docsSrc -Filter "*.md" | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $docsApp $_.Name) -Force
    $text = Get-Content -LiteralPath $_.FullName -Raw -Encoding UTF8
    if ($text -match "station-customer-readme") {
        $customerReadme = $_.FullName
    }
}
if (-not $customerReadme) { throw "docs/*.md customer readme (station-customer-readme) missing" }
Copy-Item -LiteralPath $customerReadme -Destination (Join-Path $app "README.md") -Force

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
$ico = Join-Path $PackDir "FAFUArmStation.ico"
$cscArgs = @("/nologo", "/target:winexe", "/platform:anycpu", "/utf8output", "/out:$exe", "/r:System.dll", "/r:System.Windows.Forms.dll", "/r:System.Drawing.dll")
if (Test-Path $ico) { $cscArgs += "/win32icon:$ico" }
& $csc @cscArgs $cs
if ($LASTEXITCODE -ne 0) { throw "csc failed" }
$verFile = Join-Path $app "VERSION"
$pkgVer = "1.5.0"
$initPy = Join-Path $Repo "robot_station\__init__.py"
if (Test-Path $initPy) {
    $m = Select-String -Path $initPy -Pattern '__version__\s*=\s*"([^"]+)"' | Select-Object -First 1
    if ($m) { $pkgVer = $m.Matches[0].Groups[1].Value }
}
@(
    $pkgVer,
    ("packed " + (Get-Date -Format "yyyy-MM-dd HH:mm:ss")),
    $(if ($pinocchioPy) { "pinocchio=bundled" } else { "pinocchio=skipped (station G(q)+MIT)" })
) | Set-Content -Encoding ascii $verFile

Copy-Item -Force (Join-Path $PackDir "install_shortcut.vbs") $OutDir
Copy-Item -Force (Join-Path $PackDir "FAFUArmStation.ico") $OutDir
Copy-Item -Force (Join-Path $PackDir "install_user.bat") (Join-Path $OutDir "Install.bat")
Copy-Item -Force (Join-Path $PackDir "uninstall_user.bat") (Join-Path $OutDir "Uninstall.bat")
Copy-Item -Force (Join-Path $Repo "start_live_arm.bat") (Join-Path $OutDir "StartLive.bat")
# Resolve the Chinese-named launcher via directory listing so the copy
# still works when this script is saved as UTF-8 but PowerShell reads it as another code page.
$liveBat = Get-ChildItem -LiteralPath $Repo -Filter "*.bat" |
    Where-Object { $_.Name -ne "start_live_arm.bat" -and $_.Name -like "*真机*" } |
    Select-Object -First 1
if (-not $liveBat) {
    $liveBat = Get-ChildItem -LiteralPath $Repo -Filter "*.bat" |
        Where-Object { $_.Name -ne "start_live_arm.bat" } |
        Select-Object -First 1
}
if ($liveBat) {
    Copy-Item -LiteralPath $liveBat.FullName -Destination (Join-Path $OutDir $liveBat.Name) -Force
} else {
    Write-Warning "Live-arm launcher bat not found next to the repo root"
}

$readme = @"
FAFU Arm Station — private runtime
===================================

This folder is the unpacked runtime (USB stick / debug).
The file to SEND to customers is the sibling:
  dist\FAFUArmStation.exe
They double-click that one file. They do not open this folder.

This folder is self-contained. It does NOT install Python for Windows,
does not add Python to PATH, and does not change other Python environments.

First run of the sendable exe
- Plug the arm USB into THIS PC
- Double-click dist\FAFUArmStation.exe
- It unpacks to %LOCALAPPDATA%\FAFUArmStation and opens the control window
- A desktop shortcut is created in the background
- After USB connects, the app Connects and enables the arm (motors hold pose)

If you run THIS folder instead
- Double-click FAFUArmStation.exe (or 启动真机.bat) here
- Keep this whole folder together

Later
- Double-click the desktop FAFUArmStation shortcut
- To update: send a new dist\FAFUArmStation.exe and double-click it

Uninstall: run Uninstall.bat in the user directory, or delete
%LOCALAPPDATA%\FAFUArmStation and the desktop shortcut.

USB
- Official fafu_arm_sdk must be inside app\vendor\fafu_arm_sdk
- Windows must see a COM port (USB serial driver is system-wide; this package cannot isolate kernel drivers)

Gravity
- Default build skips pinocchio. Live Gravity / Gra+Fri use station G(q)
  plus SDK apply_compensation_torque. Pass -WithPinocchio on a conda py310
  machine if you want SDK gravity_compensation_step.

Simulation
- Open the app and choose the sim arm on the right
"@
$readme | Set-Content -Encoding utf8 (Join-Path $OutDir "Readme.txt")

Write-Host ""
Write-Host "OK folder: $OutDir"
Write-Host "Launcher: $exe"
Write-Host "Python:   $pyExe"
if ($sdk) { Write-Host "SDK:      $sdkDest" } else { Write-Host "SDK:      (missing)" }
if ($pinocchioPy) { Write-Host "pinocchio: $pinocchioPy" } else { Write-Host "pinocchio: (skipped)" }

if ($SkipSingleExe) {
    Write-Host "Skipped single-exe pack (-SkipSingleExe). Folder is for USB-stick / debug."
} elseif (-not $sdk) {
    Write-Warning "SDK missing; not packing the sendable exe (live USB would not work). Place fafu_arm_sdk next to the repo and rebuild. Folder is still at $OutDir"
} else {
    $packSingle = Join-Path $PackDir "pack_single_exe.ps1"
    Write-Host "Packing sendable single exe..."
    & $packSingle -SourceDir $OutDir
    if ($LASTEXITCODE -ne 0) { throw "pack_single_exe.ps1 failed" }
    Write-Host "Send: $(Join-Path (Split-Path $OutDir) 'FAFUArmStation.exe')"
}
