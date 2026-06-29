# Package a portable install layout for distribution (no installer).
# Output:
#   dist/steam-driver/spaceoverride/     -> copy into SteamVR/drivers/
#   dist/OpenVR-SpaceOverride/           -> overlay + manifest (run -installmanifest from here)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path $PSScriptRoot -Parent
$BuildDriver = Join-Path $RepoRoot "build-overlay\Release\driver_spaceoverride.dll"
$BuildOverlay = Join-Path $RepoRoot "build-overlay\Release\OpenVR-SpaceOverride.exe"
$Manifest = Join-Path $RepoRoot "resources\manifest.vrmanifest"
$DriverManifest = Join-Path $RepoRoot "dev-resources\driver\driver.vrdrivermanifest"

$DistRoot = Join-Path $RepoRoot "dist"
$SteamDriverRoot = Join-Path $DistRoot "steam-driver\spaceoverride"
$OverlayRoot = Join-Path $DistRoot "OpenVR-SpaceOverride"

function Require-Path([string]$Path, [string]$Label) {
    if (-not (Test-Path $Path)) {
        throw "Missing $Label`: $Path`nRun: cmake --build build-overlay --config Release"
    }
}

Require-Path $BuildDriver "driver DLL"
Require-Path $BuildOverlay "overlay EXE"
Require-Path $Manifest "overlay manifest"
Require-Path $DriverManifest "driver manifest"

if (Test-Path $DistRoot) {
    Remove-Item $DistRoot -Recurse -Force
}

New-Item -ItemType Directory -Path (Join-Path $SteamDriverRoot "bin\win64") -Force | Out-Null
New-Item -ItemType Directory -Path $OverlayRoot -Force | Out-Null

Copy-Item $BuildDriver (Join-Path $SteamDriverRoot "bin\win64\driver_spaceoverride.dll") -Force
Copy-Item $DriverManifest $SteamDriverRoot -Force
Copy-Item $BuildOverlay (Join-Path $OverlayRoot "OpenVR-SpaceOverride.exe") -Force
Copy-Item $Manifest $OverlayRoot -Force

$readme = @"
OpenVR-SpaceOverride portable package (v8+)

1. Copy the entire steam-driver\spaceoverride folder to:
   Steam\steamapps\common\SteamVR\drivers\spaceoverride
   (merge/replace if it already exists)

2. Start SteamVR once, then from THIS folder (OpenVR-SpaceOverride):
   .\OpenVR-SpaceOverride.exe -installmanifest
   .\OpenVR-SpaceOverride.exe -activatemultipledrivers

3. Restart SteamVR. Open Space Override from the dashboard.
   Press Calibrate and follow on-screen instructions.

4. Verify: Settings tab -> Diagnostics should show pose hooks installed and override active.

See README.md in the repo for troubleshooting.
"@
Set-Content -Path (Join-Path $OverlayRoot "INSTALL.txt") -Value $readme -Encoding UTF8

Write-Host "Packaged to $DistRoot"
Write-Host "  steam driver: $SteamDriverRoot"
Write-Host "  overlay:      $OverlayRoot"