# Post-pack checks for the portable tree and/or the sendable single exe.
# Does NOT launch the GUI. Does NOT prove a clean Windows VM.
#
# Usage:
#   powershell -File packaging\windows\verify_portable.ps1
#   powershell -File packaging\windows\verify_portable.ps1 -SourceDir D:\path\FAFUArmStation -Exe D:\path\FAFUArmStation.exe
#   powershell -File packaging\windows\verify_portable.ps1 -AllowNoSdk

[CmdletBinding()]
param(
    [string]$SourceDir = "",
    [string]$Exe = "",
    [switch]$AllowNoSdk
)

$ErrorActionPreference = "Stop"

$PackDir = $PSScriptRoot
$Repo = (Resolve-Path (Join-Path $PackDir "..\..")).Path
if (-not $SourceDir) {
    $SourceDir = Join-Path $Repo "dist\FAFUArmStation"
}
if (-not $Exe) {
    $Exe = Join-Path $Repo "dist\FAFUArmStation.exe"
}

function Get-Sha256Hex([string]$Path) {
    $sha = [System.Security.Cryptography.SHA256]::Create()
    $stream = [System.IO.File]::OpenRead($Path)
    try {
        return ([System.BitConverter]::ToString($sha.ComputeHash($stream))).Replace("-", "")
    } finally {
        $stream.Close()
        $sha.Dispose()
    }
}

function Test-PeMz([string]$Path) {
    $fs = [System.IO.File]::OpenRead($Path)
    try {
        if ($fs.Length -lt 2) { return $false }
        $b = New-Object byte[] 2
        [void]$fs.Read($b, 0, 2)
        return ($b[0] -eq 0x4D -and $b[1] -eq 0x5A)
    } finally {
        $fs.Close()
    }
}

function Get-ExeFooter([string]$Path) {
    $fs = [System.IO.File]::OpenRead($Path)
    try {
        if ($fs.Length -lt 48) {
            throw "exe too small for FAFUEXE1 footer: $Path"
        }
        $fs.Seek(-8, [System.IO.SeekOrigin]::End) | Out-Null
        $magicBytes = New-Object byte[] 8
        [void]$fs.Read($magicBytes, 0, 8)
        $fs.Seek(-16, [System.IO.SeekOrigin]::End) | Out-Null
        $lenBytes = New-Object byte[] 8
        [void]$fs.Read($lenBytes, 0, 8)
        return @{
            Magic  = [System.Text.Encoding]::ASCII.GetString($magicBytes)
            ZipLen = [System.BitConverter]::ToInt64($lenBytes, 0)
            Length = $fs.Length
        }
    } finally {
        $fs.Close()
    }
}

$failures = New-Object System.Collections.Generic.List[string]

if (Test-Path $SourceDir) {
    Write-Host "Checking portable tree $SourceDir"
    $python = Join-Path $SourceDir "runtime\python310\python.exe"
    $thin = Join-Path $SourceDir "FAFUArmStation.exe"
    $desktop = Join-Path $SourceDir "app\station_desktop.py"
    $sdkCtrl = Join-Path $SourceDir "app\vendor\fafu_arm_sdk\fafu_robot_python\fafu_robot_controller.py"
    $placeholder = Join-Path $SourceDir "app\vendor\PUT_SDK_HERE.txt"
    if (-not (Test-Path $python)) { $failures.Add("missing $python") }
    if (-not (Test-Path $thin)) { $failures.Add("missing thin launcher $thin") }
    if (-not (Test-Path $desktop)) { $failures.Add("missing $desktop") }
    if ((Test-Path $placeholder) -and -not $AllowNoSdk) {
        $failures.Add("PUT_SDK_HERE.txt still present; do not send this package")
    }
    if (-not (Test-Path $sdkCtrl) -and -not $AllowNoSdk) {
        $failures.Add("SDK controller missing: $sdkCtrl")
    }

    if ((Test-Path $python) -and (Test-PeMz $python)) {
        Write-Host "Import check with bundled python (PATH stripped)"
        $appDir = Join-Path $SourceDir "app"
        $prevPath = $env:PATH
        $prevPyPath = $env:PYTHONPATH
        $prevHome = $env:PYTHONHOME
        $env:PATH = "$(Join-Path $SourceDir 'runtime\python310');$env:WINDIR\System32;$env:WINDIR"
        $env:PYTHONPATH = $appDir
        $env:PYTHONNOUSERSITE = "1"
        Remove-Item Env:PYTHONHOME -ErrorAction SilentlyContinue
        try {
            & $python -c "import websockets, yaml, numpy, webview, pyrealsense2; print('deps-ok')"
            if ($LASTEXITCODE -ne 0) { $failures.Add("bundled python failed to import station deps") }
            if (Test-Path $sdkCtrl) {
                $env:FAFU_ARM_SDK = (Resolve-Path (Join-Path $SourceDir "app\vendor\fafu_arm_sdk")).Path
                & $python -c "import sys; sys.path.insert(0, r'$appDir'); from robot_station.adapters.fafu_arm import import_fafu_sdk; c, m = import_fafu_sdk(); print('sdk-ok', c, m)"
                if ($LASTEXITCODE -ne 0) { $failures.Add("bundled python failed to import fafu SDK") }
            }
        } finally {
            $env:PATH = $prevPath
            if ($null -ne $prevPyPath) { $env:PYTHONPATH = $prevPyPath } else { Remove-Item Env:PYTHONPATH -ErrorAction SilentlyContinue }
            if ($null -ne $prevHome) { $env:PYTHONHOME = $prevHome }
            Remove-Item Env:FAFU_ARM_SDK -ErrorAction SilentlyContinue
        }
    } else {
        Write-Host "Skipping import check (bundled python.exe is not a PE; CI fake tree is OK)"
    }
} else {
    Write-Host "Portable tree not present: $SourceDir"
}

if (Test-Path $Exe) {
    Write-Host "Checking sendable exe $Exe"
    $footer = Get-ExeFooter $Exe
    if ($footer.Magic -ne "FAFUEXE1") {
        $failures.Add("exe magic '$($footer.Magic)' != FAFUEXE1")
    }
    if ($footer.ZipLen -lt 64 -or $footer.ZipLen -gt $footer.Length - 48) {
        $failures.Add("exe zip length $($footer.ZipLen) is invalid")
    }
    $exeHash = Get-Sha256Hex $Exe
    $manPath = Join-Path (Split-Path $Exe) "BUILD_MANIFEST.json"
    if (Test-Path $manPath) {
        $man = Get-Content -Raw -Encoding utf8 $manPath | ConvertFrom-Json
        if ([int64]$man.exe_bytes -ne [int64]$footer.Length) {
            $failures.Add("BUILD_MANIFEST.exe_bytes $($man.exe_bytes) != file $($footer.Length)")
        }
        if ($man.exe_sha256 -and ($man.exe_sha256.ToUpper() -ne $exeHash)) {
            $failures.Add("BUILD_MANIFEST.exe_sha256 does not match the exe on disk")
        }
        if ($man.payload_sha256 -and $man.exe_sha256 -and ($man.payload_sha256.ToUpper() -eq $man.exe_sha256.ToUpper())) {
            $failures.Add("BUILD_MANIFEST used the zip hash for both payload and exe; they must differ")
        }
        if ($man.magic -and $man.magic -ne "FAFUEXE1") {
            $failures.Add("BUILD_MANIFEST.magic $($man.magic) != FAFUEXE1")
        }
        Write-Host "Manifest matches exe_bytes=$($footer.Length) exe_sha256=$exeHash"
    } else {
        Write-Warning "BUILD_MANIFEST.json missing next to exe (pack_single_exe.ps1 should write it)"
    }
} else {
    Write-Host "Sendable exe not present: $Exe"
}

if ($failures.Count -gt 0) {
    Write-Host ""
    Write-Host "verify_portable FAILED:"
    foreach ($f in $failures) { Write-Host " - $f" }
    exit 1
}

Write-Host "verify_portable OK"
exit 0
