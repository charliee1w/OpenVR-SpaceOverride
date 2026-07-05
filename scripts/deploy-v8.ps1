# Deploy vanilla-v8 build artifacts to all runtime locations.
# Requires admin for Program Files copies.
#
# Build first:
#   cmake -B build-vanilla -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-vanilla --config Release
#
# Restart SteamVR after deploy.
param(
    [string]$BuildDir = (Join-Path $PSScriptRoot '..\build-vanilla\Release')
)

$ErrorActionPreference = 'Stop'
$root = Resolve-Path (Join-Path $PSScriptRoot '..')
$build = Resolve-Path $BuildDir

$dllSrc = Join-Path $build 'driver_spaceoverride.dll'
$exeSrc = Join-Path $build 'OpenVR-SpaceOverride.exe'
if (-not (Test-Path $dllSrc)) { throw "Missing $dllSrc - run: cmake --build build-vanilla --config Release" }
if (-not (Test-Path $exeSrc)) { throw "Missing $exeSrc" }

$targets = @(
    (Join-Path $root '..\..\bin\win64\driver_spaceoverride.dll'),
    (Join-Path $root 'bin\driver\bin\win64\driver_spaceoverride.dll'),
    'C:\Program Files\OpenVR-SpaceOverride\driver\bin\win64\driver_spaceoverride.dll'
)

foreach ($t in $targets) {
    $dir = Split-Path $t -Parent
    if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    Copy-Item $dllSrc $t -Force
    Write-Host "DLL -> $t"
}

$exeTargets = @(
    (Join-Path $root 'bin\OpenVR-SpaceOverride.exe'),
    'C:\Program Files\OpenVR-SpaceOverride\OpenVR-SpaceOverride.exe'
)
foreach ($t in $exeTargets) {
    Copy-Item $exeSrc $t -Force
    Write-Host "EXE -> $t"
}

$ref = (Get-FileHash $dllSrc).Hash
$exeRef = (Get-FileHash $exeSrc).Hash
$ok = $true
foreach ($t in $targets) {
    if ((Get-FileHash $t).Hash -ne $ref) { Write-Error "Hash mismatch: $t"; $ok = $false }
}
foreach ($t in $exeTargets) {
    if ((Get-FileHash $t).Hash -ne $exeRef) { Write-Error "Hash mismatch: $t"; $ok = $false }
}
if ($ok) { Write-Host 'All v8 artifacts verified.' }