# Build, verify, and package a public release (installer EXE + portable ZIP).
$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path $PSScriptRoot -Parent
$VersionFile = Join-Path $RepoRoot "include\shared\Version.h"
$Version = "8.3.0"
if (Test-Path $VersionFile) {
    $raw = Get-Content $VersionFile -Raw
    if ($raw -match '#define\s+SPACECAL_VERSION_STRING\s+"([^"]+)"') {
        $Version = $Matches[1]
    }
}

$BinRoot = Join-Path $RepoRoot "bin"
$ReleaseDir = Join-Path $RepoRoot "release"
$InstallerName = "SpaceOverride_Installer_$Version.exe"
$ZipName = "OpenVR-SpaceOverride-$Version-win64.zip"
$Makensis = "C:\Program Files (x86)\NSIS\makensis.exe"
if (-not (Test-Path $Makensis)) {
    $Makensis = "C:\Program Files\NSIS\makensis.exe"
}

Push-Location $RepoRoot
try {
    Write-Host "OpenVR-SpaceOverride release build v$Version"
    Write-Host ""

    & (Join-Path $PSScriptRoot "build-release.ps1")
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    & (Join-Path $PSScriptRoot "smoke-test-steamvr.ps1")
    if ($LASTEXITCODE -ne 0) {
        Write-Host "FAIL smoke test — aborting release package"
        exit 1
    }

    Write-Host ""
    Write-Host "Staging installer bin/ layout..."

    $BuildDriver = Join-Path $RepoRoot "build-overlay\Release\driver_spaceoverride.dll"
    $BuildOverlay = Join-Path $RepoRoot "build-overlay\Release\OpenVR-SpaceOverride.exe"
    $OpenVrApiDll = Join-Path $RepoRoot "3rdparty\OpenVR\bin\win64\openvr_api.dll"

    foreach ($p in @($BuildDriver, $BuildOverlay, $OpenVrApiDll)) {
        if (-not (Test-Path $p)) { throw "Missing build artifact: $p" }
    }

    if (Test-Path $BinRoot) { Remove-Item $BinRoot -Recurse -Force }
    New-Item -ItemType Directory -Path (Join-Path $BinRoot "driver\bin\win64") -Force | Out-Null
    New-Item -ItemType Directory -Path (Join-Path $BinRoot "driver\resources\settings") -Force | Out-Null

    Copy-Item $BuildOverlay (Join-Path $BinRoot "OpenVR-SpaceOverride.exe") -Force
    Copy-Item $OpenVrApiDll (Join-Path $BinRoot "openvr_api.dll") -Force
    Copy-Item (Join-Path $RepoRoot "resources\icon.png") $BinRoot -Force
    Copy-Item (Join-Path $RepoRoot "resources\manifest.vrmanifest") $BinRoot -Force
    Copy-Item (Join-Path $RepoRoot "resources\LICENSE.txt") $BinRoot -Force
    Copy-Item (Join-Path $RepoRoot "resources\LICENSE") $BinRoot -Force
    Copy-Item (Join-Path $RepoRoot "resources\LICENSES") $BinRoot -Force
    Copy-Item $BuildDriver (Join-Path $BinRoot "driver\bin\win64\driver_spaceoverride.dll") -Force
    Copy-Item (Join-Path $RepoRoot "dev-resources\driver\driver.vrdrivermanifest") (Join-Path $BinRoot "driver") -Force
    Copy-Item (Join-Path $RepoRoot "dev-resources\driver\resources\driver.vrresources") (Join-Path $BinRoot "driver\resources") -Force
    Copy-Item (Join-Path $RepoRoot "dev-resources\driver\resources\settings\default.vrsettings") (Join-Path $BinRoot "driver\resources\settings") -Force

    if (-not (Test-Path $Makensis)) {
        throw "NSIS makensis not found. Install: winget install NSIS.NSIS"
    }

    Write-Host "Compiling NSIS installer..."
    & $Makensis "/DAPP_VERSION=$Version" "/DAPP_VERSION_META=$Version.0" (Join-Path $RepoRoot "dev-resources\installer.nsi")
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

    $InstallerExe = Join-Path $RepoRoot "dev-resources\SpaceOverride_Installer_$Version.exe"
    if (-not (Test-Path $InstallerExe)) {
        throw "Installer EXE not produced at $InstallerExe"
    }

    if (Test-Path $ReleaseDir) { Remove-Item $ReleaseDir -Recurse -Force }
    New-Item -ItemType Directory -Path $ReleaseDir -Force | Out-Null

    $FinalInstaller = Join-Path $ReleaseDir $InstallerName
    Copy-Item $InstallerExe $FinalInstaller -Force

    $PortableStage = Join-Path $ReleaseDir "portable"
    Copy-Item (Join-Path $RepoRoot "dist") $PortableStage -Recurse -Force
    Compress-Archive -Path (Join-Path $PortableStage "*") -DestinationPath (Join-Path $ReleaseDir $ZipName) -Force
    Remove-Item $PortableStage -Recurse -Force

    $installerHash = (Get-FileHash $FinalInstaller -Algorithm SHA256).Hash.Substring(0, 16)
    $zipHash = (Get-FileHash (Join-Path $ReleaseDir $ZipName) -Algorithm SHA256).Hash.Substring(0, 16)

    Write-Host ""
    Write-Host "Release package ready:"
    Write-Host "  $FinalInstaller"
    Write-Host "    SHA256: $installerHash"
    Write-Host "  $(Join-Path $ReleaseDir $ZipName)"
    Write-Host "    SHA256: $zipHash"
    Write-Host ""
    Write-Host "Publish: gh release create v$Version $FinalInstaller $(Join-Path $ReleaseDir $ZipName) --title `"v$Version`" --notes-file CHANGELOG.md"
}
finally {
    Pop-Location
}