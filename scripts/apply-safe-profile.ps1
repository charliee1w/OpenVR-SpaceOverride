# Apply conservative Space Override profile settings (VD/Quest + lighthouse).
$ErrorActionPreference = 'Stop'
$regPath = 'HKCU:\Software\Classes\Local Settings\Software\OpenVR-SpaceOverride'
if (-not (Test-Path $regPath)) {
    Write-Host 'No Space Override profile found.'
    exit 1
}
$json = (Get-ItemProperty $regPath).Config
if ([string]::IsNullOrWhiteSpace($json)) {
    Write-Host 'Profile empty — calibrate in overlay first.'
    exit 1
}
$obj = $json | ConvertFrom-Json
if ($obj -is [array]) { $p = $obj[0] } else { $p = $obj }
$p | Add-Member -NotePropertyName predictionAuto -NotePropertyValue $false -Force
$p | Add-Member -NotePropertyName predictionTime -NotePropertyValue 2.0 -Force
$p | Add-Member -NotePropertyName eAngVel -NotePropertyValue $true -Force
$p | Add-Member -NotePropertyName headFilterEnabled -NotePropertyValue $false -Force
$p | Add-Member -NotePropertyName tundraMode -NotePropertyValue $false -Force
$p | Add-Member -NotePropertyName autoPartialRecalOnMountDrift -NotePropertyValue $false -Force
$out = if ($obj -is [array]) { @($p) | ConvertTo-Json -Depth 8 -Compress } else { $p | ConvertTo-Json -Depth 8 -Compress }
Set-ItemProperty -Path $regPath -Name Config -Value $out
Write-Host 'Applied safe profile: prediction manual 2f, smoothing off, tundra/auto-recal modes off'
Write-Host 'Restart SteamVR, then open Space Override once to re-apply driver state.'