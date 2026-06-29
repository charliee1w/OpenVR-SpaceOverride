# Re-enable spaceoverride driver after a fixed build is deployed.

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path $PSScriptRoot -Parent
$BuildDriverDll = Join-Path $RepoRoot "build-driver\Release\driver_spaceoverride.dll"

$SteamDriverRoot = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\spaceoverride"
$SteamDriverDll = Join-Path $SteamDriverRoot "bin\win64\driver_spaceoverride.dll"
$SteamDriverDllDisabled = "$SteamDriverDll.disabled"
$SteamDriverManifest = Join-Path $SteamDriverRoot "driver.vrdrivermanifest"
$SteamDriverManifestDisabled = "$SteamDriverManifest.disabled"

$DevManifest = Join-Path $RepoRoot "dev-resources\driver\driver.vrdrivermanifest"

if (-not (Test-Path $BuildDriverDll)) {
    throw "Build driver first: $BuildDriverDll"
}

if (Test-Path $SteamDriverDllDisabled) {
    Rename-Item -Path $SteamDriverDllDisabled -NewName "driver_spaceoverride.dll" -Force
}
Copy-Item -Path $BuildDriverDll -Destination $SteamDriverDll -Force

if (Test-Path $SteamDriverManifestDisabled) {
    Remove-Item $SteamDriverManifestDisabled -Force -ErrorAction SilentlyContinue
}
Copy-Item -Path $DevManifest -Destination $SteamDriverManifest -Force

Write-Host "Enabled spaceoverride driver. Restart SteamVR."