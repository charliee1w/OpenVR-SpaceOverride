# Remove stale Program Files OpenVR-SpaceOverride install and repoint SteamVR to portable dist.
# Keeps the current driver in SteamVR\drivers\spaceoverride (deployed build).

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path $PSScriptRoot -Parent
$DistDir = Join-Path $RepoRoot "dist\OpenVR-SpaceOverride"
$DistExe = Join-Path $DistDir "OpenVR-SpaceOverride.exe"
$DistManifest = Join-Path $DistDir "manifest.vrmanifest"
$PfRoot = "C:\Program Files\OpenVR-SpaceOverride"
$PfExe = Join-Path $PfRoot "OpenVR-SpaceOverride.exe"
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

Write-Host "OpenVR-SpaceOverride stale install removal"
Write-Host ""

if (-not (Test-Path $DistExe)) {
    throw "Portable dist missing. Run deploy-steamvr.ps1 first."
}

Stop-SteamVR

Write-Host "--- External driver registration ---"
if (Test-Path $VrPathReg) {
    foreach ($target in @("spaceoverride", (Join-Path $PfRoot "driver"))) {
        & $VrPathReg removedriver $target 2>$null
    }
    Write-Host "  OK removed stale vrpathreg entries pointing at Program Files"
    & $VrPathReg show
}
else {
    Write-Host "  SKIP vrpathreg not found"
}

Write-Host ""
Write-Host "--- Overlay manifest paths (appconfig.json) ---"
if (Test-Path $AppConfig) {
    $json = Get-Content $AppConfig -Raw | ConvertFrom-Json
    $distManifestEscaped = $DistManifest -replace '\\', '\\'
    $before = @($json.manifest_paths)
    $json.manifest_paths = @(
        $json.manifest_paths | Where-Object {
            $_ -notmatch 'OpenVR-SpaceOverride' -and $_ -notmatch 'OpenVR-Spaceoverride'
        }
    )
    if ($json.manifest_paths -notcontains $DistManifest) {
        $json.manifest_paths += $DistManifest
    }
    $json | ConvertTo-Json -Depth 6 | Set-Content $AppConfig -Encoding UTF8
    Write-Host "  OK repointed overlay manifest to:"
    Write-Host "     $DistManifest"
    if ($before -match 'Program Files\\OpenVR-SpaceOverride') {
        Write-Host "  OK removed Program Files manifest path"
    }
}
else {
    Write-Host "  WARN appconfig.json not found"
}

Write-Host ""
Write-Host "--- Program Files install ---"
if (Test-Path $PfRoot) {
    if (Test-Path (Join-Path $PfRoot "Uninstall.exe")) {
        Write-Host "  Running Uninstall.exe (silent)..."
        $uninstall = Start-Process -FilePath (Join-Path $PfRoot "Uninstall.exe") -ArgumentList "/S" -Wait -PassThru -Verb RunAs -ErrorAction SilentlyContinue
        if ($uninstall -and $uninstall.ExitCode -eq 0) {
            Write-Host "  OK uninstaller completed"
        }
        else {
            Write-Host "  WARN uninstaller exit $($uninstall.ExitCode) - trying direct removal"
        }
    }
    if (Test-Path $PfRoot) {
        Write-Host "  Removing $PfRoot ..."
        Start-Process pwsh -ArgumentList "-NoProfile -Command Remove-Item -LiteralPath '$PfRoot' -Recurse -Force" -Verb RunAs -Wait | Out-Null
        if (-not (Test-Path $PfRoot)) {
            Write-Host "  OK Program Files install removed"
        }
        else {
            Write-Host "  WARN still present - re-run this script as Administrator"
        }
    }
}
else {
    Write-Host "  OK Program Files install not present"
}

Write-Host ""
Write-Host "--- Workspace stray copy ---"
if (Test-Path $WsExe) {
    Remove-Item $WsExe -Force
    Write-Host "  OK removed stale workspace copy: $WsExe"
}
else {
    Write-Host "  OK no workspace stray copy"
}

Write-Host ""
Write-Host "--- Steam driver (kept) ---"
$dll = Join-Path $SteamDriverRoot "bin\win64\driver_spaceoverride.dll"
if (Test-Path $dll) {
    $hash = (Get-FileHash $dll -Algorithm SHA256).Hash.Substring(0, 16)
    Write-Host "  OK $dll ($hash)"
}
else {
    Write-Host "  WARN driver DLL missing - run deploy-steamvr.ps1"
}

Write-Host ""
Write-Host "Done. Next time SteamVR starts:"
Write-Host "  1. Overlay autolaunch should use dist build"
Write-Host "  2. Run from dist if needed:"
Write-Host "       cd `"$DistDir`""
Write-Host "       .\OpenVR-SpaceOverride.exe -installmanifest"
Write-Host "       .\OpenVR-SpaceOverride.exe -activatemultipledrivers"