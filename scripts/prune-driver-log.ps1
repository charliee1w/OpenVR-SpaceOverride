# Remove or rotate oversized space_calibrator_driver.log files (SteamVR must be stopped).
param(
    [double]$WarnMb = 100,
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$paths = @(
    "$env:LOCALAPPDATA\openvr\logs\space_calibrator_driver.log",
    "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\space_calibrator_driver.log",
    "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\space_calibrator_driver.log"
)

if (Get-Process -Name vrserver,vrcompositor -ErrorAction SilentlyContinue) {
    Write-Host "SteamVR is running. Stop SteamVR before pruning driver logs." -ForegroundColor Red
    exit 1
}

$removed = 0
foreach ($path in $paths) {
    if (-not (Test-Path $path)) { continue }

    $item = Get-Item $path
    $sizeMb = [math]::Round($item.Length / 1MB, 1)
    if ($sizeMb -lt $WarnMb -and -not $Force) {
        Write-Host "OK   $path ($sizeMb MB)" -ForegroundColor Green
        continue
    }

    $backup = "$path.pruned-$(Get-Date -Format 'yyyyMMdd-HHmmss')"
    if ($sizeMb -ge 1024) {
        Write-Host "DELETE $path ($sizeMb MB) - legacy spam log" -ForegroundColor Yellow
        Remove-Item $path -Force
    }
    else {
        Write-Host "RENAME $path ($sizeMb MB) -> $backup" -ForegroundColor Yellow
        Move-Item $path $backup
    }
    $removed++
}

foreach ($path in $paths) {
    $old = "$path.old"
    if (Test-Path $old) {
        $sizeMb = [math]::Round((Get-Item $old).Length / 1MB, 1)
        if ($sizeMb -ge $WarnMb) {
            Write-Host "DELETE $old ($sizeMb MB)" -ForegroundColor Yellow
            Remove-Item $old -Force
            $removed++
        }
    }
}

if ($removed -eq 0) {
    Write-Host "No oversized driver logs found." -ForegroundColor Green
}
else {
    Write-Host "Pruned $removed log file(s). New builds log to %LOCALAPPDATA%\openvr\logs\" -ForegroundColor Green
}