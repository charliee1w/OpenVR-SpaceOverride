# Deploy OpenVR-SpaceOverride fork build artifacts and register with SteamVR.
# Run after every build. SteamVR loads the driver from the Steam drivers folder
# (not Program Files) when both are present - see deploy targets below.

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path $PSScriptRoot -Parent
$BuildDriverDll = Join-Path $RepoRoot "build-overlay\Release\driver_spaceoverride.dll"
$BuildOverlayExe = Join-Path $RepoRoot "build-overlay\Release\OpenVR-SpaceOverride.exe"
if (-not (Test-Path $BuildDriverDll)) {
    $BuildDriverDll = Join-Path $RepoRoot "build-driver\Release\driver_spaceoverride.dll"
}

# SteamVR scans runtime\drivers\ first; this is the DLL vrserver actually loads.
$SteamDriverDll = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\spaceoverride\bin\win64\driver_spaceoverride.dll"
$SteamDriverManifest = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\spaceoverride\driver.vrdrivermanifest"
$SteamDriverManifestDisabled = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\spaceoverride\driver.vrdrivermanifest.disabled"

$InstallRoot = "C:\Program Files\OpenVR-SpaceOverride"
$InstallDriverDll = Join-Path $InstallRoot "driver\bin\win64\driver_spaceoverride.dll"
$InstallOverlayExe = Join-Path $InstallRoot "OpenVR-SpaceOverride.exe"
$InstallDriverManifest = Join-Path $InstallRoot "driver\driver.vrdrivermanifest"
$InstallOverlayManifest = Join-Path $InstallRoot "manifest.vrmanifest"

$VrPathReg = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"
$DriverLog = Join-Path $env:LOCALAPPDATA "openvr\logs\space_calibrator_driver.log"
$LegacyDriverLog = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\space_calibrator_driver.log"

function Require-Path([string]$Path, [string]$Label) {
    if (-not (Test-Path $Path)) {
        throw "Missing $Label`: $Path`nBuild first (driver + overlay Release configs)."
    }
}

function Copy-Artifact([string]$Source, [string]$Destination, [switch]$Optional) {
    $destDir = Split-Path $Destination -Parent
    if (-not (Test-Path $destDir)) {
        New-Item -ItemType Directory -Path $destDir -Force | Out-Null
    }
    try {
        Copy-Item -Path $Source -Destination $Destination -Force -ErrorAction Stop
    }
    catch {
        if ($Optional) {
            Write-Host "  SKIP (needs admin): $Destination"
            return $false
        }
        throw
    }
    $hash = (Get-FileHash $Destination -Algorithm SHA256).Hash.Substring(0, 16)
    $time = (Get-Item $Destination).LastWriteTime.ToString("yyyy-MM-dd HH:mm:ss")
    Write-Host "  OK $time | $hash | $Destination"
    return $true
}

function Test-SteamVRRunning {
    return $null -ne (Get-Process -Name "vrserver" -ErrorAction SilentlyContinue)
}

Write-Host "OpenVR-SpaceOverride deploy"
Write-Host "Repo: $RepoRoot"
Write-Host ""

Require-Path $BuildDriverDll "driver build output"
Require-Path $BuildOverlayExe "overlay build output"

if (Test-SteamVRRunning) {
    Write-Host "NOTE: SteamVR is running. Driver DLL may be locked until you restart SteamVR."
}

Write-Host "Copying driver DLL (SteamVR load path is required; Program Files is backup):"
Copy-Artifact $BuildDriverDll $SteamDriverDll | Out-Null
$pfDriverCopied = Copy-Artifact $BuildDriverDll $InstallDriverDll -Optional

if (Test-Path $InstallDriverManifest) {
    Copy-Item -Path $InstallDriverManifest -Destination $SteamDriverManifest -Force -ErrorAction Stop
    if (Test-Path $SteamDriverManifestDisabled) {
        Remove-Item $SteamDriverManifestDisabled -Force
    }
    Write-Host "  OK enabled driver manifest at Steam drivers folder"
}

Write-Host ""
Write-Host "Copying overlay EXE:"
$pfOverlayCopied = Copy-Artifact $BuildOverlayExe $InstallOverlayExe -Optional
if (-not $pfDriverCopied -or -not $pfOverlayCopied) {
    Write-Host ""
    Write-Host "Program Files install needs elevation — trying admin sync..."
    $adminSync = Join-Path $PSScriptRoot "sync-program-files-admin.ps1"
    if (Test-Path $adminSync) {
        try {
            Start-Process -FilePath "pwsh" -ArgumentList "-NoProfile -ExecutionPolicy Bypass -File `"$adminSync`"" -Verb RunAs -Wait
            $pfDriverCopied = Test-Path $InstallDriverDll -and ((Get-FileHash $BuildDriverDll -Algorithm SHA256).Hash -eq (Get-FileHash $InstallDriverDll -Algorithm SHA256).Hash)
            $pfOverlayCopied = Test-Path $InstallOverlayExe -and ((Get-FileHash $BuildOverlayExe -Algorithm SHA256).Hash -eq (Get-FileHash $InstallOverlayExe -Algorithm SHA256).Hash)
            if ($pfDriverCopied -and $pfOverlayCopied) {
                Write-Host "  OK Program Files install updated via admin sync"
            }
        }
        catch {
            Write-Host "  SKIP admin sync: $_"
        }
    }
    if (-not $pfDriverCopied -or -not $pfOverlayCopied) {
        Write-Host "Re-run as Administrator: pwsh -File `"$adminSync`""
    }
}

Write-Host ""
Write-Host "Registering external driver path:"
if (-not (Test-Path $VrPathReg)) {
    throw "vrpathreg.exe not found: $VrPathReg"
}
try {
    & $VrPathReg removedriver "spaceoverride" 2>$null
    & $VrPathReg removedriver (Join-Path $InstallRoot "driver") 2>$null
    if ($pfDriverCopied -and (Test-Path $InstallDriverManifest)) {
        & $VrPathReg adddriver (Join-Path $InstallRoot "driver")
        if ($LASTEXITCODE -ne 0) { throw "exit code $LASTEXITCODE" }
        Write-Host "  OK external driver registered (Program Files)"
    }
    else {
        Write-Host "  OK using Steam drivers folder only (no stale PF vrpathreg)"
        Write-Host "     $((Split-Path $SteamDriverDll -Parent | Split-Path -Parent | Split-Path -Parent))"
    }
}
catch {
    Write-Host "  SKIP vrpathreg (may need admin): $_"
}

$steamRunning = Test-SteamVRRunning
Write-Host ""
if ($steamRunning -and (Test-Path $InstallOverlayExe)) {
    Write-Host "SteamVR is running - registering overlay autolaunch:"
    Push-Location $InstallRoot
    try {
        & $InstallOverlayExe -installmanifest
        if ($LASTEXITCODE -ne 0) { throw "-installmanifest failed" }
        Write-Host "  OK overlay manifest + autolaunch"

        & $InstallOverlayExe -activatemultipledrivers
        if ($LASTEXITCODE -ne 0) { throw "-activatemultipledrivers failed" }
        Write-Host "  OK activateMultipleDrivers"
    }
    finally {
        Pop-Location
    }
}
elseif ($pfOverlayCopied -and (Test-Path $InstallOverlayExe)) {
    Write-Host "SteamVR is not running — registering overlay manifest from Program Files:"
    Push-Location $InstallRoot
    try {
        & $InstallOverlayExe -installmanifest
        if ($LASTEXITCODE -ne 0) { throw "-installmanifest failed" }
        Write-Host "  OK overlay manifest + autolaunch"

        & $InstallOverlayExe -activatemultipledrivers
        if ($LASTEXITCODE -ne 0) { throw "-activatemultipledrivers failed" }
        Write-Host "  OK activateMultipleDrivers"
    }
    finally {
        Pop-Location
    }
}
else {
    Write-Host "SteamVR is not running."
    Write-Host "  Skipped -installmanifest and -activatemultipledrivers."
    Write-Host "  Start SteamVR once, then re-run this script OR run from $InstallRoot :"
    Write-Host "    .\OpenVR-SpaceOverride.exe -installmanifest"
    Write-Host "    .\OpenVR-SpaceOverride.exe -activatemultipledrivers"
}

Write-Host ""
Write-Host "Verification:"
$buildHash = (Get-FileHash $BuildDriverDll -Algorithm SHA256).Hash
$steamHash = (Get-FileHash $SteamDriverDll -Algorithm SHA256).Hash

$exitCode = 0
if ($buildHash -eq $steamHash) {
    Write-Host "  OK Steam driver DLL matches build (this is what vrserver loads)"
}
else {
    Write-Host "  FAIL Steam driver DLL hash mismatch - restart SteamVR and re-run deploy"
    $exitCode = 1
}
if (Test-Path $InstallDriverDll) {
    $installHash = (Get-FileHash $InstallDriverDll -Algorithm SHA256).Hash
    if ($buildHash -eq $installHash) {
        Write-Host "  OK Program Files driver DLL matches build"
    }
    else {
        Write-Host "  WARN Program Files driver DLL stale - re-run script as admin"
    }
}

foreach ($logPath in @($DriverLog, $LegacyDriverLog)) {
    if (-not (Test-Path $logPath)) { continue }
    $tail = Get-Content $logPath -Tail 100 -ErrorAction SilentlyContinue
    $lastLoad = $tail | Select-String -Pattern "OpenVR-SpaceOverride .+ loaded" | Select-Object -Last 1
    if ($lastLoad) {
        Write-Host "  Last driver log ($logPath): $($lastLoad.Line.Trim())"
        break
    }
}

Write-Host ""
Write-Host "On SteamVR start, vrserver should load:"
Write-Host "  $SteamDriverDll"
Write-Host "Overlay autolaunches from:"
Write-Host "  $InstallOverlayManifest"
Write-Host ""
Write-Host "Updating portable dist/ package:"
& (Join-Path $PSScriptRoot "package-portable.ps1")

$PortableOverlay = Join-Path $RepoRoot "dist\OpenVR-SpaceOverride\OpenVR-SpaceOverride.exe"
$PortableDir = Join-Path $RepoRoot "dist\OpenVR-SpaceOverride"
if ((Test-Path $InstallOverlayExe) -and (Test-Path $InstallOverlayManifest)) {
    Write-Host "  Registering overlay autolaunch from Program Files install:"
    Push-Location $InstallRoot
    try {
        & $InstallOverlayExe -installmanifest
        if ($LASTEXITCODE -eq 0) {
            Write-Host "  OK Program Files overlay manifest registered"
            & $InstallOverlayExe -activatemultipledrivers
        }
        else {
            Write-Host "  WARN -installmanifest returned $LASTEXITCODE"
        }
    }
    finally {
        Pop-Location
    }
}
elseif (Test-Path $PortableOverlay) {
    Write-Host ""
    Write-Host "Program Files overlay unavailable — registering autolaunch from portable dist:"
    Push-Location $PortableDir
    try {
        & $PortableOverlay -installmanifest
        if ($LASTEXITCODE -eq 0) {
            Write-Host "  OK portable overlay manifest registered"
            & $PortableOverlay -activatemultipledrivers
        }
        else {
            Write-Host "  WARN -installmanifest returned $LASTEXITCODE (SteamVR may need to be running once)"
        }
    }
    finally {
        Pop-Location
    }
}

Write-Host ""
Write-Host "Done. Restart SteamVR if it was already running when you deployed."
exit $exitCode