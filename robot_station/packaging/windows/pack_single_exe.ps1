# Wrap dist\FAFUArmStation\ into one sendable exe:
#   dist\FAFUArmStation.exe  = SingleFileLauncher stub + zip + 48-byte footer
#
# The customer double-clicks that file. It unpacks to
# %LOCALAPPDATA%\FAFUArmStation and starts the thin launcher there.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File packaging\windows\pack_single_exe.ps1
#   powershell -File packaging\windows\pack_single_exe.ps1 -SourceDir D:\path\FAFUArmStation
#   powershell -File packaging\windows\pack_single_exe.ps1 -AllowNoSdk

[CmdletBinding()]
param(
    [string]$SourceDir = "",
    [string]$OutExe = "",
    [switch]$AllowNoSdk
)

$ErrorActionPreference = "Stop"

$PackDir = $PSScriptRoot
$Repo = (Resolve-Path (Join-Path $PackDir "..\..")).Path
$Cache = Join-Path $PackDir "cache"
if (-not $SourceDir) {
    $SourceDir = Join-Path $Repo "dist\FAFUArmStation"
}
if (-not $OutExe) {
    $OutExe = Join-Path $Repo "dist\FAFUArmStation.exe"
}

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

function Get-NetDir {
    $csc = Get-Csc
    return Split-Path $csc
}

if (-not (Test-Path $SourceDir)) {
    throw "Portable folder missing: $SourceDir  (run build_portable.ps1 first)"
}
$python = Join-Path $SourceDir "runtime\python310\python.exe"
$thin = Join-Path $SourceDir "FAFUArmStation.exe"
$desktop = Join-Path $SourceDir "app\station_desktop.py"
if (-not (Test-Path $python)) { throw "missing $python" }
if (-not (Test-Path $thin)) { throw "missing thin launcher $thin" }
if (-not (Test-Path $desktop)) { throw "missing $desktop" }

$sdkCtrl = Join-Path $SourceDir "app\vendor\fafu_arm_sdk\fafu_robot_python\fafu_robot_controller.py"
$placeholder = Join-Path $SourceDir "app\vendor\PUT_SDK_HERE.txt"
if (-not (Test-Path $sdkCtrl)) {
    $msg = "SDK not inside the portable folder. Live USB will not work. Rebuild without -SkipSdk, with fafu_arm_sdk next to the repo."
    if ($AllowNoSdk) {
        Write-Warning $msg
    } else {
        throw $msg
    }
}
if ((Test-Path $placeholder) -and -not $AllowNoSdk) {
    throw "Package still has PUT_SDK_HERE.txt. Do not send this exe for live USB."
}

New-Item -ItemType Directory -Force -Path $Cache | Out-Null
New-Item -ItemType Directory -Force -Path (Split-Path $OutExe) | Out-Null

$payload = Join-Path $Cache "single-exe-payload.zip"
if (Test-Path $payload) { Remove-Item -Force $payload }

Write-Host "Zipping $SourceDir"
Add-Type -AssemblyName System.IO.Compression.FileSystem
[System.IO.Compression.ZipFile]::CreateFromDirectory(
    (Resolve-Path $SourceDir).Path,
    $payload,
    [System.IO.Compression.CompressionLevel]::Optimal,
    $false
)
$zipLen = (Get-Item $payload).Length
if ($zipLen -lt 64) { throw "payload zip is empty" }
Write-Host ("Payload zip: {0:N1} MB" -f ($zipLen / 1MB))

$sha = [System.Security.Cryptography.SHA256]::Create()
$zipStream = [System.IO.File]::OpenRead($payload)
try {
    $hashBytes = $sha.ComputeHash($zipStream)
} finally {
    $zipStream.Close()
    $sha.Dispose()
}
$hashHex = ([System.BitConverter]::ToString($hashBytes)).Replace("-", "")
Write-Host "Payload SHA-256: $hashHex"

Write-Host "Compiling single-file stub"
$csc = Get-Csc
$netDir = Get-NetDir
$stub = Join-Path $Cache "FAFUArmStation-stub.exe"
$cs = Join-Path $PackDir "SingleFileLauncher.cs"
$ico = Join-Path $PackDir "FAFUArmStation.ico"
$cscArgs = @(
    "/nologo", "/target:winexe", "/platform:anycpu", "/utf8output",
    "/out:$stub",
    "/r:System.dll",
    "/r:System.Windows.Forms.dll",
    "/r:System.Drawing.dll",
    "/r:$(Join-Path $netDir 'System.IO.Compression.dll')",
    "/r:$(Join-Path $netDir 'System.IO.Compression.FileSystem.dll')"
)
if (Test-Path $ico) { $cscArgs += "/win32icon:$ico" }
& $csc @cscArgs $cs
if ($LASTEXITCODE -ne 0) { throw "csc SingleFileLauncher.cs failed" }

Write-Host "Writing $OutExe"
$outStream = [System.IO.File]::Create($OutExe)
try {
    $stubStream = [System.IO.File]::OpenRead($stub)
    try { $stubStream.CopyTo($outStream) } finally { $stubStream.Close() }
    $zipStream = [System.IO.File]::OpenRead($payload)
    try { $zipStream.CopyTo($outStream) } finally { $zipStream.Close() }
    $outStream.Write($hashBytes, 0, 32)
    $lenBytes = [System.BitConverter]::GetBytes([int64]$zipLen)
    $outStream.Write($lenBytes, 0, 8)
    $magic = [System.Text.Encoding]::ASCII.GetBytes("FAFUEXE1")
    $outStream.Write($magic, 0, 8)
} finally {
    $outStream.Close()
}

$finalLen = (Get-Item $OutExe).Length
$expect = (Get-Item $stub).Length + $zipLen + 48
if ($finalLen -ne $expect) {
    throw "output size $finalLen != stub+zip+footer $expect"
}

$exeSha = [System.Security.Cryptography.SHA256]::Create()
$exeStream = [System.IO.File]::OpenRead($OutExe)
try {
    $exeHashBytes = $exeSha.ComputeHash($exeStream)
} finally {
    $exeStream.Close()
    $exeSha.Dispose()
}
$exeHashHex = ([System.BitConverter]::ToString($exeHashBytes)).Replace("-", "")

$version = "unknown"
$verFile = Join-Path $SourceDir "app\VERSION"
if (Test-Path $verFile) {
    $version = ((Get-Content -TotalCount 1 -Encoding utf8 $verFile) -as [string]).Trim()
}

$manifest = [ordered]@{
    version         = $version
    built           = (Get-Date).ToString("s")
    exe             = [System.IO.Path]::GetFileName($OutExe)
    exe_bytes       = $finalLen
    exe_sha256      = $exeHashHex
    payload_sha256  = $hashHex
    zip_bytes       = $zipLen
    magic           = "FAFUEXE1"
    sendable_exe    = (Resolve-Path $OutExe).Path
}
$manifestPath = Join-Path (Split-Path $OutExe) "BUILD_MANIFEST.json"
($manifest | ConvertTo-Json) | Set-Content -Encoding utf8 $manifestPath
Write-Host "Manifest: $manifestPath"
Write-Host "Exe SHA-256: $exeHashHex"

$verify = Join-Path $PackDir "verify_portable.ps1"
if (Test-Path $verify) {
    if ($AllowNoSdk) {
        & $verify -SourceDir $SourceDir -Exe $OutExe -AllowNoSdk
    } else {
        & $verify -SourceDir $SourceDir -Exe $OutExe
    }
    if ($LASTEXITCODE -ne 0) { throw "verify_portable.ps1 failed" }
}

Write-Host ""
Write-Host "OK: $OutExe"
Write-Host ("Size: {0:N1} MB" -f ($finalLen / 1MB))
Write-Host "Send THIS file only. Customer double-clicks it (USB plugged in)."
Write-Host "It unpacks to %LOCALAPPDATA%\FAFUArmStation and opens the control window."
