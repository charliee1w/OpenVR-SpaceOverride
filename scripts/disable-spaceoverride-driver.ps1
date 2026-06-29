# Remove spaceoverride from SteamVR driver load path (manifest + DLL rename).
# Safe recovery when driver_spaceoverride.dll crashes vrserver on Init.

$ErrorActionPreference = "Stop"

$VrPathReg = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"
$SteamDriverRoot = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\spaceoverride"
$SteamDriverDll = Join-Path $SteamDriverRoot "bin\win64\driver_spaceoverride.dll"
$SteamDriverDllDisabled = "$SteamDriverDll.disabled"
$SteamDriverManifest = Join-Path $SteamDriverRoot "driver.vrdrivermanifest"
$SteamDriverManifestDisabled = "$SteamDriverManifest.disabled"

$InstallRoot = "C:\Program Files\OpenVR-SpaceOverride"
$InstallDriverDll = Join-Path $InstallRoot "driver\bin\win64\driver_spaceoverride.dll"
$InstallDriverDllDisabled = "$InstallDriverDll.disabled"
$InstallDriverManifest = Join-Path $InstallRoot "driver\driver.vrdrivermanifest"
$InstallDriverManifestDisabled = "$InstallDriverManifest.disabled"

function Disable-Manifest([string]$Active, [string]$Disabled) {
    if (Test-Path $Active) {
        if (Test-Path $Disabled) { Remove-Item $Disabled -Force }
        Rename-Item -Path $Active -NewName (Split-Path $Disabled -Leaf) -Force
        Write-Host "  OK disabled manifest: $Active"
        return
    }
    if (Test-Path $Disabled) {
        Write-Host "  OK manifest already disabled: $Disabled"
        return
    }
    Write-Host "  SKIP no manifest at $Active"
}

function Disable-DriverDll([string]$Active, [string]$Disabled) {
    if (-not (Test-Path $Active)) {
        if (Test-Path $Disabled) {
            Write-Host "  OK DLL already disabled: $Disabled"
        }
        else {
            Write-Host "  SKIP no DLL at $Active"
        }
        return
    }
    if (Test-Path $Disabled) { Remove-Item $Disabled -Force }
    Rename-Item -Path $Active -NewName (Split-Path $Disabled -Leaf) -Force
    Write-Host "  OK disabled DLL: $Active"
}

Write-Host "Disabling OpenVR-SpaceOverride SteamVR driver..."
Write-Host ""

$vrProcs = @("vrserver", "vrmonitor", "vrcompositor", "vrstartup", "vrwebhelper")
$running = Get-Process -Name $vrProcs -ErrorAction SilentlyContinue
if ($running) {
    Write-Host "Stopping SteamVR processes..."
    $running | Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 2
}

Write-Host "Steam drivers folder:"
Disable-Manifest $SteamDriverManifest $SteamDriverManifestDisabled
Disable-DriverDll $SteamDriverDll $SteamDriverDllDisabled

Write-Host ""
Write-Host "Program Files install:"
try {
    Disable-Manifest $InstallDriverManifest $InstallDriverManifestDisabled
    Disable-DriverDll $InstallDriverDll $InstallDriverDllDisabled
}
catch {
    Write-Host "  WARN Program Files paths need admin: $_"
    Write-Host "  Re-run this script as Administrator if PF install still active."
}

if (Test-Path $VrPathReg) {
    Write-Host ""
    Write-Host "Removing external driver registration (best-effort):"
    foreach ($name in @("spaceoverride", "OpenVR-SpaceOverride")) {
        & $VrPathReg removedriver $name 2>$null
    }
    & $VrPathReg removedriver (Join-Path $InstallRoot "driver") 2>$null
    Write-Host "  OK vrpathreg cleanup attempted"
    & $VrPathReg
}

Write-Host ""
Write-Host "Done. Start SteamVR - vrserver should load VirtualDesktop -> vrlink -> lighthouse without spaceoverride."
Write-Host "Re-enable with enable-spaceoverride-driver.ps1 after fixing the driver build."