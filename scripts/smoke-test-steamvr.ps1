# Post-deploy smoke checks for OpenVR-SpaceOverride (no headset required).
# Verifies build artifacts, Steam load path, manifest, and registry profile readability.

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path $PSScriptRoot -Parent
$BuildDriverDll = Join-Path $RepoRoot "build-overlay\Release\driver_spaceoverride.dll"
$BuildOverlayExe = Join-Path $RepoRoot "build-overlay\Release\OpenVR-SpaceOverride.exe"
$SteamDriverDll = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\spaceoverride\bin\win64\driver_spaceoverride.dll"
$SteamManifest = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\spaceoverride\driver.vrdrivermanifest"
$InstallOverlay = "C:\Program Files\OpenVR-SpaceOverride\OpenVR-SpaceOverride.exe"
$PortableOverlay = Join-Path $RepoRoot "dist\OpenVR-SpaceOverride\OpenVR-SpaceOverride.exe"
$RegistryKey = "HKCU:\Software\Classes\Local Settings\Software\OpenVR-SpaceOverride"

$failures = @()

function Test-Exists([string]$Path, [string]$Label) {
    if (-not (Test-Path $Path)) {
        $script:failures += "Missing $Label`: $Path"
        Write-Host "FAIL $Label" -ForegroundColor Red
        return $false
    }
    Write-Host "OK   $Label" -ForegroundColor Green
    return $true
}

function Test-HashMatch([string]$A, [string]$B, [string]$Label) {
    if (-not ((Test-Path $A) -and (Test-Path $B))) {
        $script:failures += "Hash skip (missing file): $Label"
        return
    }
    $ha = (Get-FileHash $A -Algorithm SHA256).Hash
    $hb = (Get-FileHash $B -Algorithm SHA256).Hash
    if ($ha -ne $hb) {
        $script:failures += "$Label hash mismatch (build vs installed)"
        Write-Host "FAIL $Label hash mismatch" -ForegroundColor Red
        Write-Host "  build:     $ha"
        Write-Host "  installed: $hb"
    }
    else {
        Write-Host "OK   $Label hash match" -ForegroundColor Green
    }
}

Write-Host "OpenVR-SpaceOverride smoke test"
Write-Host ""

Write-Host "Build outputs:"
Test-Exists $BuildDriverDll "build driver DLL" | Out-Null
Test-Exists $BuildOverlayExe "build overlay EXE" | Out-Null

Write-Host ""
Write-Host "SteamVR install paths:"
Test-Exists $SteamDriverDll "Steam driver DLL" | Out-Null
Test-Exists $SteamManifest "driver manifest" | Out-Null
$buildOverlayHash = (Get-FileHash $BuildOverlayExe -Algorithm SHA256).Hash
$overlayOk = $false

if (Test-Path $PortableOverlay) {
    $portableHash = (Get-FileHash $PortableOverlay -Algorithm SHA256).Hash
    if ($buildOverlayHash -eq $portableHash) {
        Write-Host "OK   Portable dist overlay matches build" -ForegroundColor Green
        $overlayOk = $true
    }
    else {
        Write-Host "FAIL Portable dist overlay stale" -ForegroundColor Red
        $failures += "Portable dist overlay hash mismatch"
    }
}
else {
    Write-Host "WARN Portable dist overlay missing (run deploy-steamvr.ps1)" -ForegroundColor Yellow
}

if (Test-Path $InstallOverlay) {
    $pfOverlayHash = (Get-FileHash $InstallOverlay -Algorithm SHA256).Hash
    if ($buildOverlayHash -eq $pfOverlayHash) {
        Write-Host "OK   Program Files overlay matches build" -ForegroundColor Green
        $overlayOk = $true
    }
    else {
        Write-Host "WARN Program Files overlay stale (use portable dist or admin deploy)" -ForegroundColor Yellow
        if (-not $overlayOk) {
            $failures += "No overlay path matches build (PF stale, portable missing or stale)"
        }
    }
}
else {
    Write-Host "WARN Program Files overlay missing (deploy may need admin)" -ForegroundColor Yellow
    if (-not $overlayOk) {
        $failures += "No overlay install found"
    }
}

Write-Host ""
Write-Host "Artifact freshness:"
Test-HashMatch $BuildDriverDll $SteamDriverDll "driver DLL"

Write-Host ""
Write-Host "Registry profile:"
try {
    $config = Get-ItemProperty -Path $RegistryKey -Name Config -ErrorAction Stop
    if ($config.Config -and $config.Config.Length -gt 10) {
        Write-Host "OK   registry profile present ($($config.Config.Length) chars)" -ForegroundColor Green
    }
    else {
        Write-Host "WARN registry profile empty - user needs calibration" -ForegroundColor Yellow
    }
}
catch {
    Write-Host "WARN registry key missing - first run or profile wiped" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "Overlay manifest registration:"
$AppConfig = "C:\Program Files (x86)\Steam\config\appconfig.json"
$CanonicalManifest = "C:\Program Files\OpenVR-SpaceOverride\manifest.vrmanifest"
if (Test-Path $AppConfig) {
    $paths = (Get-Content $AppConfig -Raw | ConvertFrom-Json).manifest_paths
    $spacePaths = @($paths | Where-Object { $_ -match 'OpenVR-SpaceOverride' })
    if ($spacePaths.Count -eq 1 -and $spacePaths[0] -eq $CanonicalManifest) {
        Write-Host "OK   appconfig uses Program Files manifest" -ForegroundColor Green
    }
    elseif ($spacePaths.Count -eq 0) {
        Write-Host "WARN no SpaceOverride manifest in appconfig" -ForegroundColor Yellow
    }
    else {
        Write-Host "WARN stale/duplicate SpaceOverride manifest paths:" -ForegroundColor Yellow
        $spacePaths | ForEach-Object { Write-Host "       $_" }
        $failures += "appconfig has stale SpaceOverride manifest registration"
    }
}
else {
    Write-Host "WARN appconfig.json missing" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "SteamVR process:"
if (Get-Process -Name "vrserver" -ErrorAction SilentlyContinue) {
    Write-Host "NOTE SteamVR is running - restart to load new driver DLL if just deployed" -ForegroundColor Yellow
}
else {
    Write-Host "OK   SteamVR not running" -ForegroundColor Green
}

Write-Host ""
if ($failures.Count -gt 0) {
    Write-Host ('SMOKE TEST FAILED ({0} issues):' -f $failures.Count) -ForegroundColor Red
    $failures | ForEach-Object { Write-Host "  - $_" }
    exit 1
}

Write-Host "SMOKE TEST PASSED" -ForegroundColor Green
exit 0