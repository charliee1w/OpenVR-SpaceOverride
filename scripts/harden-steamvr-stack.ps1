# Apply log-driven hardening for SpaceOverride + SteamVR stack. SteamVR must be stopped.
$ErrorActionPreference = 'Stop'

$RepoRoot = Split-Path $PSScriptRoot -Parent
$SteamVrSettings = 'C:\Program Files (x86)\Steam\config\steamvr.vrsettings'
$AppConfig = 'C:\Program Files (x86)\Steam\config\appconfig.json'
$VrPathReg = 'C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe'
$InstallRoot = 'C:\Program Files\OpenVR-SpaceOverride'
$InstallOverlayExe = Join-Path $InstallRoot 'OpenVR-SpaceOverride.exe'
$InstallOverlayManifest = Join-Path $InstallRoot 'manifest.vrmanifest'
$CanonicalManifest = $InstallOverlayManifest
$DistManifest = Join-Path $RepoRoot 'dist\OpenVR-SpaceOverride\manifest.vrmanifest'
$SteamDriverRoot = 'C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\spaceoverride'
$SteamDriverManifest = Join-Path $SteamDriverRoot 'driver.vrdrivermanifest'
$SteamDriverManifestDisabled = "$SteamDriverManifest.disabled"
$WorkspaceStrayExe = Join-Path (Split-Path $RepoRoot -Parent) 'OpenVR-SpaceOverride.exe'
$BackupRoot = Join-Path $env:LOCALAPPDATA 'OpenVR-SpaceOverride\stack-hardening-backup'
$Stamp = Get-Date -Format 'yyyyMMdd-HHmmss'

function Write-Step([string]$Message) {
    Write-Host $Message -ForegroundColor Cyan
}

function Write-Ok([string]$Message) {
    Write-Host "  OK $Message" -ForegroundColor Green
}

function Write-Warn([string]$Message) {
    Write-Host "  WARN $Message" -ForegroundColor Yellow
}

if (Get-Process -Name vrserver, vrcompositor, vrmonitor -ErrorAction SilentlyContinue) {
    Write-Host 'Stop SteamVR first.' -ForegroundColor Red
    exit 1
}

New-Item -ItemType Directory -Path (Join-Path $BackupRoot $Stamp) -Force | Out-Null
$backupDir = Join-Path $BackupRoot $Stamp

function Backup-File([string]$Path) {
    if (Test-Path $Path) {
        Copy-Item $Path (Join-Path $backupDir ([IO.Path]::GetFileName($Path))) -Force
    }
}

Write-Step 'Hardening SteamVR + SpaceOverride stack...'

# 1. steamvr.vrsettings — ghost drivers and debug noise
if (Test-Path $SteamVrSettings) {
    Backup-File $SteamVrSettings
    $json = Get-Content $SteamVrSettings -Raw | ConvertFrom-Json
    $changed = $false
    if ($json.PSObject.Properties.Name -contains 'driver_01spacecalibrator') {
        $json.PSObject.Properties.Remove('driver_01spacecalibrator')
        $changed = $true
        Write-Ok 'removed ghost driver_01spacecalibrator'
    }
    if ($json.steamvr.debugInputBinding -eq $true) {
        $json.steamvr.debugInputBinding = $false
        $changed = $true
        Write-Ok 'disabled debugInputBinding'
    }
    if ($changed) {
        $json | ConvertTo-Json -Depth 20 | Set-Content $SteamVrSettings -Encoding UTF8
    }
    else {
        Write-Ok 'steamvr.vrsettings already clean'
    }
}

# 2. appconfig.json — one canonical SpaceOverride manifest
if (Test-Path $AppConfig) {
    Backup-File $AppConfig
    $json = Get-Content $AppConfig -Raw | ConvertFrom-Json
    $before = @($json.manifest_paths)
    $nonSpace = @(
        $json.manifest_paths | Where-Object {
            $_ -notmatch 'OpenVR-SpaceOverride' -and $_ -notmatch 'OpenVR-Spaceoverride'
        }
    )

    $targetManifest = $null
    if ((Test-Path $InstallOverlayExe) -and (Test-Path $CanonicalManifest)) {
        $targetManifest = $CanonicalManifest
    }
    elseif (Test-Path $DistManifest) {
        $targetManifest = $DistManifest
        Write-Warn 'Program Files overlay missing — using portable dist manifest'
    }

    $json.manifest_paths = $nonSpace
    if ($targetManifest) {
        $json.manifest_paths += $targetManifest
    }

    $removed = $before.Count - $nonSpace.Count
    if ($removed -gt 0 -or ($targetManifest -and $before -notcontains $targetManifest)) {
        $json | ConvertTo-Json -Depth 6 | Set-Content $AppConfig -Encoding UTF8
        Write-Ok "appconfig manifest -> $targetManifest"
        if ($removed -gt 0) {
            Write-Ok "removed $removed stale SpaceOverride manifest path(s)"
        }
    }
    else {
        Write-Ok 'appconfig manifest paths already canonical'
    }
}

# 3. vrpathreg — broken or duplicate external driver entries
if (Test-Path $VrPathReg) {
    Backup-File (Join-Path $env:LOCALAPPDATA 'openvr\openvrpaths.vrpath')
    $show = & $VrPathReg show 2>&1 | Out-String
    $lines = $show -split "`n" | Where-Object { $_ -match ':\s+' }
    foreach ($line in $lines) {
        if ($line -notmatch '^\s*(.+?)\s*:\s*(.+)\s*$') { continue }
        $name = $Matches[1].Trim()
        $path = $Matches[2].Trim()
        if ($name -notmatch 'NO DRIVER NAME FOUND' -and $path -notmatch 'Standable Full Body Estimation\\?$') { continue }
        if (-not (Test-Path (Join-Path $path 'driver.vrdrivermanifest'))) {
            & $VrPathReg removedriver $path 2>$null | Out-Null
            Write-Ok "removed broken vrpathreg entry: $path"
        }
    }

    # Steam drivers folder is canonical; drop stale PF duplicate registration.
    $pfDriver = Join-Path $InstallRoot 'driver'
    if ((Test-Path (Join-Path $SteamDriverRoot 'driver.vrdrivermanifest')) -and (Test-Path $pfDriver)) {
        & $VrPathReg removedriver $pfDriver 2>$null | Out-Null
        Write-Ok 'removed stale Program Files vrpathreg driver (Steam folder is canonical)'
    }
}

# 4. Driver manifest must be enabled
if (Test-Path $SteamDriverManifestDisabled) {
    if (-not (Test-Path $SteamDriverManifest)) {
        Move-Item $SteamDriverManifestDisabled $SteamDriverManifest -Force
        Write-Ok 're-enabled driver.vrdrivermanifest'
    }
    else {
        Remove-Item $SteamDriverManifestDisabled -Force
        Write-Ok 'removed duplicate driver.vrdrivermanifest.disabled'
    }
}

# 5. Legacy driver logs + workspace stray copy
Write-Step 'Pruning legacy driver logs...'
& (Join-Path $PSScriptRoot 'prune-driver-log.ps1') -Force -LegacyOnly | Out-Host

if (Test-Path $WorkspaceStrayExe) {
    Remove-Item $WorkspaceStrayExe -Force
    Write-Ok "removed workspace stray copy: $WorkspaceStrayExe"
}

# 6. Re-register overlay manifest from canonical install
if ((Test-Path $InstallOverlayExe) -and (Test-Path $InstallOverlayManifest)) {
    Push-Location $InstallRoot
    try {
        & $InstallOverlayExe -installmanifest | Out-Null
        if ($LASTEXITCODE -eq 0) {
            Write-Ok 'overlay manifest registered from Program Files'
            & $InstallOverlayExe -activatemultipledrivers | Out-Null
        }
        else {
            Write-Warn "-installmanifest returned $LASTEXITCODE"
        }
    }
    finally {
        Pop-Location
    }
}

# 7. Verify deploy
Write-Step 'Running smoke test...'
& (Join-Path $RepoRoot 'scripts\smoke-test-steamvr.ps1') | Out-Host
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host ''
Write-Host "Backup: $backupDir"
Write-Host 'Next: start SteamVR, use normally, exit, then run scripts\analyze-vr-logs.ps1'