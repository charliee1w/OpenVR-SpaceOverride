# Apply log-driven hardening for SpaceOverride + SteamVR stack. SteamVR must be stopped.
$ErrorActionPreference = 'Stop'

$RepoRoot = Split-Path $PSScriptRoot -Parent
$SteamVrSettings = 'C:\Program Files (x86)\Steam\config\steamvr.vrsettings'
$BackupRoot = Join-Path $env:LOCALAPPDATA 'OpenVR-SpaceOverride\stack-hardening-backup'
$Stamp = Get-Date -Format 'yyyyMMdd-HHmmss'

if (Get-Process -Name vrserver,vrcompositor,vrmonitor -ErrorAction SilentlyContinue) {
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

Write-Host 'Hardening SteamVR + SpaceOverride stack...' -ForegroundColor Cyan

# 1. Remove ghost Space Calibrator driver priority from vrsettings
if (Test-Path $SteamVrSettings) {
    Backup-File $SteamVrSettings
    $json = Get-Content $SteamVrSettings -Raw | ConvertFrom-Json
    $changed = $false
    if ($json.PSObject.Properties.Name -contains 'driver_01spacecalibrator') {
        $json.PSObject.Properties.Remove('driver_01spacecalibrator')
        $changed = $true
        Write-Host 'Removed ghost driver_01spacecalibrator from steamvr.vrsettings'
    }
    if ($json.steamvr.debugInputBinding -eq $true) {
        $json.steamvr.debugInputBinding = $false
        $changed = $true
        Write-Host 'Disabled debugInputBinding'
    }
    if ($changed) {
        $json | ConvertTo-Json -Depth 20 | Set-Content $SteamVrSettings -Encoding UTF8
    }
}

# 2. Prune misleading stale driver logs (old builds wrote to SteamVR folder)
& (Join-Path $PSScriptRoot 'prune-driver-log.ps1') -Force | Out-Host

# 3. Verify deploy
& (Join-Path $RepoRoot 'scripts\smoke-test-steamvr.ps1') | Out-Host

Write-Host ''
Write-Host 'Backup: ' $backupDir
Write-Host 'Next SteamVR start: run analyze-vr-logs.ps1 after a session to verify.'