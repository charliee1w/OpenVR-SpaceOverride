# One-shot elevated sync into Program Files install (called by deploy-steamvr).
$ErrorActionPreference = 'Stop'
$Repo = Split-Path $PSScriptRoot -Parent
$BuildDriver = Join-Path $Repo 'build-overlay\Release\driver_spaceoverride.dll'
$BuildOverlay = Join-Path $Repo 'build-overlay\Release\OpenVR-SpaceOverride.exe'
$InstallRoot = 'C:\Program Files\OpenVR-SpaceOverride'
$DriverManifest = Join-Path $Repo 'dev-resources\driver\driver.vrdrivermanifest'
$OverlayManifest = Join-Path $Repo 'resources\manifest.vrmanifest'
$OpenVrApi = Join-Path $Repo '3rdparty\OpenVR\bin\win64\openvr_api.dll'
$Icon = Join-Path $Repo 'resources\icon.png'

$items = @(
    @{ Src = $BuildDriver; Dst = Join-Path $InstallRoot 'driver\bin\win64\driver_spaceoverride.dll' },
    @{ Src = $BuildOverlay; Dst = Join-Path $InstallRoot 'OpenVR-SpaceOverride.exe' },
    @{ Src = $DriverManifest; Dst = Join-Path $InstallRoot 'driver\driver.vrdrivermanifest' },
    @{ Src = $OverlayManifest; Dst = Join-Path $InstallRoot 'manifest.vrmanifest' },
    @{ Src = $OpenVrApi; Dst = Join-Path $InstallRoot 'openvr_api.dll' },
    @{ Src = $Icon; Dst = Join-Path $InstallRoot 'icon.png' }
)

foreach ($item in $items) {
    $dir = Split-Path $item.Dst -Parent
    if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }
    Copy-Item $item.Src $item.Dst -Force
    $h = (Get-FileHash $item.Dst -Algorithm SHA256).Hash.Substring(0, 16)
    Write-Output "OK $h $($item.Dst)"
}

Push-Location $InstallRoot
try {
    & (Join-Path $InstallRoot 'OpenVR-SpaceOverride.exe') -installmanifest
    & (Join-Path $InstallRoot 'OpenVR-SpaceOverride.exe') -activatemultipledrivers
}
finally {
    Pop-Location
}