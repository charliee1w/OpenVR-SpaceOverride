# One-shot build + package for strangers (Release, tests, portable dist).
$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path $PSScriptRoot -Parent
Push-Location $RepoRoot
try {
    if (-not (Test-Path "build-overlay")) {
        cmake -B build-overlay -DBUILD_OVERLAY=ON -DBUILD_DRIVER=ON -DBUILD_TESTS=ON
    }
    cmake --build build-overlay --config Release
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    ctest --test-dir build-overlay -C Release --output-on-failure
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    & (Join-Path $PSScriptRoot "package-portable.ps1")
    Write-Host ""
    Write-Host "Release build complete. Portable package: dist\"
    Write-Host "Next: copy dist\steam-driver\spaceoverride to SteamVR\drivers\, then follow dist\OpenVR-SpaceOverride\INSTALL.txt"
}
finally {
    Pop-Location
}