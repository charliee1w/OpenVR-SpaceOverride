# Diagnose common OpenVR-SpaceOverride failure modes (no headset required).
$ErrorActionPreference = "Continue"

$RepoRoot = Split-Path $PSScriptRoot -Parent
$BuildOverlay = Join-Path $RepoRoot "build-overlay\Release\OpenVR-SpaceOverride.exe"
$BuildDriver = Join-Path $RepoRoot "build-overlay\Release\driver_spaceoverride.dll"
$SteamDriver = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\spaceoverride\bin\win64\driver_spaceoverride.dll"
$PfOverlay = "C:\Program Files\OpenVR-SpaceOverride\OpenVR-SpaceOverride.exe"
$WorkspaceOverlay = Join-Path (Split-Path $RepoRoot -Parent) "OpenVR-SpaceOverride.exe"
$RegistryKey = "HKCU:\Software\Classes\Local Settings\Software\OpenVR-SpaceOverride"
$Logs = @(
    "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\space_calibrator_driver.log",
    "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\space_calibrator_driver.log",
    "$env:LOCALAPPDATA\openvr\logs\space_calibrator_driver.log"
)

function Hash-Short([string]$Path) {
    if (-not (Test-Path $Path)) { return $null }
    return (Get-FileHash $Path -Algorithm SHA256).Hash.Substring(0, 16)
}

Write-Host "=== OpenVR-SpaceOverride debug report ===" -ForegroundColor Cyan
Write-Host ""

# 1. SteamVR running
$vr = Get-Process -Name vrserver,vrcompositor -ErrorAction SilentlyContinue
if ($vr) {
    Write-Host '[WARN] SteamVR is running - restart after deploy to load new driver DLL' -ForegroundColor Yellow
    $vr | ForEach-Object { Write-Host "       $($_.Name) pid=$($_.Id)" }
}
else {
    Write-Host '[OK]   SteamVR not running' -ForegroundColor Green
}

# 2. Artifact hashes
Write-Host ""
Write-Host "--- Binaries ---"
$buildDriverHash = Hash-Short $BuildDriver
$steamDriverHash = Hash-Short $SteamDriver
$buildOverlayHash = Hash-Short $BuildOverlay
$pfOverlayHash = Hash-Short $PfOverlay
$wsOverlayHash = Hash-Short $WorkspaceOverlay

if ($buildDriverHash -eq $steamDriverHash) {
    Write-Host "[OK]   Driver DLL matches build ($buildDriverHash)" -ForegroundColor Green
}
else {
    Write-Host '[FAIL] Driver DLL stale or missing' -ForegroundColor Red
    Write-Host "       build: $BuildDriver -> $buildDriverHash"
    Write-Host "       steam: $SteamDriver -> $steamDriverHash"
}

if ($buildOverlayHash -eq $pfOverlayHash) {
    Write-Host "[OK]   Program Files overlay matches build ($buildOverlayHash)" -ForegroundColor Green
}
else {
    Write-Host '[FAIL] Program Files overlay STALE - SteamVR autolaunch uses old EXE' -ForegroundColor Red
    Write-Host "       build:     $buildOverlayHash ($(if (Test-Path $BuildOverlay) { (Get-Item $BuildOverlay).LastWriteTime }))"
    Write-Host "       PF install: $pfOverlayHash ($(if (Test-Path $PfOverlay) { (Get-Item $PfOverlay).LastWriteTime }))"
    Write-Host "       Fix: re-run deploy-steamvr.ps1 as Administrator"
}

if ($buildOverlayHash -eq $wsOverlayHash) {
    Write-Host "[OK]   Workspace overlay matches build" -ForegroundColor Green
}
elseif (Test-Path $WorkspaceOverlay) {
    Write-Host "[WARN] Workspace overlay differs from build" -ForegroundColor Yellow
}

# 3. Registry profile
Write-Host ""
Write-Host "--- Registry profile ---"
try {
    $config = (Get-ItemProperty -Path $RegistryKey -Name Config -ErrorAction Stop).Config
    if ($config -and $config.Length -gt 20) {
        Write-Host "[OK]   Profile present ($($config.Length) chars)" -ForegroundColor Green
        if ($config -match '"tracker_serial"') { Write-Host "       contains tracker_serial" }
        if ($config -match '"rel_qw"') { Write-Host "       contains mount offset" }
    }
    else {
        Write-Host '[FAIL] Profile EMPTY - override inactive until calibration' -ForegroundColor Red
        Write-Host "       Cause: likely wiped by pre-fix quit bug, or never calibrated"
        Write-Host "       Fix: run Calibrate in overlay, or Import profile JSON in Settings > Diagnostics"
    }
}
catch {
    Write-Host '[FAIL] Registry key missing - no saved profile' -ForegroundColor Red
}

# 4. Driver logs
Write-Host ""
Write-Host "--- Driver logs ---"
foreach ($log in $Logs) {
    if (-not (Test-Path $log)) { continue }
    $sizeMb = [math]::Round((Get-Item $log).Length / 1MB, 1)
    $color = if ($sizeMb -gt 100) { "Yellow" } else { "Gray" }
    Write-Host "[$sizeMb MB] $log" -ForegroundColor $color

    if ($sizeMb -gt 100) {
        Write-Host "       WARN oversized log - run scripts/prune-driver-log.ps1 (SteamVR stopped)" -ForegroundColor Yellow
    }

    $tail = Get-Content $log -Tail 200 -ErrorAction SilentlyContinue
    $loaded = $tail | Select-String -Pattern 'OpenVR-SpaceOverride .+ loaded' | Select-Object -Last 1
    $hook = $tail | Select-String -Pattern 'Enabled hook for IVRServerDriverHost' | Select-Object -Last 1
    $ipcErr = $tail | Select-String -Pattern 'CreateNamedPipe failed|ResponseError' | Select-Object -Last 3

    if ($loaded) { Write-Host "       last load: $($loaded.Line.Trim())" }
    if ($hook) { Write-Host "       hooks: $($hook.Line.Trim())" -ForegroundColor Green }
    if ($ipcErr) {
        Write-Host "       recent IPC issues:" -ForegroundColor Yellow
        $ipcErr | ForEach-Object { Write-Host "         $($_.Line.Trim())" }
    }
}

Write-Host ""
Write-Host "--- Expected Diagnostics when working ---"
Write-Host "  Pose hooks: installed"
Write-Host "  Driver override: active"
Write-Host "  Override: active"
Write-Host "  Runtime mount residual: < 30 mm (static)"
Write-Host ""
Write-Host "Run smoke-test-steamvr.ps1 for deploy verification."