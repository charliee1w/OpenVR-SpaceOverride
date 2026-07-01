# Remove or rotate oversized space_calibrator_driver.log files (SteamVR must be stopped).
param(
    [double]$WarnMb = 100,
    [switch]$Force,
    [switch]$LegacyOnly
)

$ErrorActionPreference = "Stop"

$PrimaryLog = "$env:LOCALAPPDATA\openvr\logs\space_calibrator_driver.log"
$LegacyLogs = @(
    "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\space_calibrator_driver.log",
    "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\space_calibrator_driver.log"
)

if (Get-Process -Name vrserver,vrcompositor -ErrorAction SilentlyContinue) {
    Write-Host "SteamVR is running. Stop SteamVR before pruning driver logs." -ForegroundColor Red
    exit 1
}

function Prune-LogFile([string]$Path, [switch]$AlwaysIfPresent) {
    if (-not (Test-Path $Path)) { return 0 }

    $item = Get-Item $Path
    $sizeMb = [math]::Round($item.Length / 1MB, 1)
    $shouldPrune = $AlwaysIfPresent -or ($Force -and $LegacyOnly) -or ($sizeMb -ge $WarnMb)
    if (-not $shouldPrune) {
        Write-Host "OK   $Path ($sizeMb MB)" -ForegroundColor Green
        return 0
    }

    if ($sizeMb -ge 1024) {
        Write-Host "DELETE $Path ($sizeMb MB) - legacy spam log" -ForegroundColor Yellow
        Remove-Item $Path -Force
    }
    else {
        $backup = "$Path.pruned-$(Get-Date -Format 'yyyyMMdd-HHmmss')"
        Write-Host "RENAME $Path ($sizeMb MB) -> $backup" -ForegroundColor Yellow
        Move-Item $Path $backup
    }
    return 1
}

$removed = 0

if (-not $LegacyOnly) {
    $removed += Prune-LogFile $PrimaryLog
}

foreach ($path in $LegacyLogs) {
    # Legacy SteamVR-folder logs are always stale for current builds; remove on -Force.
    $removed += Prune-LogFile $path -AlwaysIfPresent:($Force -or $LegacyOnly)
    $old = "$path.old"
    if (Test-Path $old) {
        $sizeMb = [math]::Round((Get-Item $old).Length / 1MB, 1)
        if ($Force -or $LegacyOnly -or $sizeMb -ge $WarnMb) {
            Write-Host "DELETE $old ($sizeMb MB)" -ForegroundColor Yellow
            Remove-Item $old -Force
            $removed++
        }
    }
}

if ($removed -eq 0) {
    Write-Host "No driver logs needed pruning." -ForegroundColor Green
}
else {
    Write-Host "Pruned $removed log file(s). Active log: $PrimaryLog" -ForegroundColor Green
}