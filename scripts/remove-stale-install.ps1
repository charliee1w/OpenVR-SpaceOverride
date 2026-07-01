# Remove stale duplicate SpaceOverride installs and registrations.
# Keeps Program Files overlay (canonical) and SteamVR\drivers\spaceoverride driver.
# Use harden-steamvr-stack.ps1 for routine cleanup; this script is for deep reset.

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path $PSScriptRoot -Parent
$PfRoot = "C:\Program Files\OpenVR-SpaceOverride"
$CanonicalManifest = Join-Path $PfRoot "manifest.vrmanifest"
$DistDir = Join-Path $RepoRoot "dist\OpenVR-SpaceOverride"
$DistManifest = Join-Path $DistDir "manifest.vrmanifest"
$WsExe = Join-Path (Split-Path $RepoRoot -Parent) "OpenVR-SpaceOverride.exe"
$VrPathReg = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"
$AppConfig = "C:\Program Files (x86)\Steam\config\appconfig.json"
$SteamDriverRoot = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\spaceoverride"

function Stop-SteamVR {
    $names = @("vrserver", "vrmonitor", "vrcompositor", "vrstartup", "vrwebhelper", "OpenVR-SpaceOverride")
    $procs = Get-Process -Name $names -ErrorAction SilentlyContinue
    if ($procs) {
        Write-Host "Stopping SteamVR / overlay processes..."
        $procs | Stop-Process -Force -ErrorAction SilentlyContinue
        Start-Sleep -Seconds 2
    }
}

Write-Host "OpenVR-SpaceOverride stale duplicate cleanup"
Write-Host ""

Stop-SteamVR

Write-Host "--- vrpathreg ---"
if (Test-Path $VrPathReg) {
    & $VrPathReg removedriver (Join-Path $PfRoot "driver") 2>$null
    & $VrPathReg removedriver "C:\Program Files (x86)\Steam\steamapps\common\Standable Full Body Estimation" 2>$null
    Write-Host "  OK removed stale external driver registrations"
    & $VrPathReg show
}

Write-Host ""
Write-Host "--- appconfig.json ---"
if (Test-Path $AppConfig) {
    $json = Get-Content $AppConfig -Raw | ConvertFrom-Json
    $nonSpace = @(
        $json.manifest_paths | Where-Object {
            $_ -notmatch 'OpenVR-SpaceOverride' -and $_ -notmatch 'OpenVR-Spaceoverride'
        }
    )
    $target = if (Test-Path $CanonicalManifest) { $CanonicalManifest } else { $DistManifest }
    $json.manifest_paths = $nonSpace + @($target)
    $json | ConvertTo-Json -Depth 6 | Set-Content $AppConfig -Encoding UTF8
    Write-Host "  OK canonical manifest: $target"
}

Write-Host ""
Write-Host "--- Legacy driver logs ---"
& (Join-Path $PSScriptRoot "prune-driver-log.ps1") -Force -LegacyOnly

Write-Host ""
Write-Host "--- Workspace stray copy ---"
if (Test-Path $WsExe) {
    Remove-Item $WsExe -Force
    Write-Host "  OK removed $WsExe"
}
else {
    Write-Host "  OK no workspace stray copy"
}

Write-Host ""
Write-Host "--- Kept installs ---"
$dll = Join-Path $SteamDriverRoot "bin\win64\driver_spaceoverride.dll"
if (Test-Path $dll) {
    $hash = (Get-FileHash $dll -Algorithm SHA256).Hash.Substring(0, 16)
    Write-Host "  driver: $dll ($hash)"
}
if (Test-Path (Join-Path $PfRoot "OpenVR-SpaceOverride.exe")) {
    Write-Host "  overlay: $PfRoot"
}
elseif (Test-Path (Join-Path $DistDir "OpenVR-SpaceOverride.exe")) {
    Write-Host "  overlay (portable): $DistDir"
}

Write-Host ""
Write-Host "Done. Run harden-steamvr-stack.ps1 to verify."