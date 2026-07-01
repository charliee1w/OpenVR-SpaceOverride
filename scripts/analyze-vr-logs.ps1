# Analyze latest SteamVR + SpaceOverride logs for stability risks (no headset required).
$ErrorActionPreference = 'Continue'

$VrServerLog = 'C:\Program Files (x86)\Steam\logs\vrserver.txt'
$CompositorLog = 'C:\Program Files (x86)\Steam\logs\vrcompositor.txt'
$DriverLogs = @(
    "$env:LOCALAPPDATA\openvr\logs\space_calibrator_driver.log",
    'C:\Program Files (x86)\Steam\steamapps\common\SteamVR\space_calibrator_driver.log',
    'C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\space_calibrator_driver.log'
)
$SteamVrSettings = 'C:\Program Files (x86)\Steam\config\steamvr.vrsettings'
$RegistryKey = 'HKCU:\Software\Classes\Local Settings\Software\OpenVR-SpaceOverride'

function Write-Finding([string]$Level, [string]$Message) {
    $color = switch ($Level) {
        'OK' { 'Green' }
        'WARN' { 'Yellow' }
        'FAIL' { 'Red' }
        default { 'Gray' }
    }
    Write-Host "[$Level] $Message" -ForegroundColor $color
}

Write-Host '=== VR session log analysis ===' -ForegroundColor Cyan
Write-Host ''

# --- Driver log ---
$primaryDriverLog = $DriverLogs | Where-Object { Test-Path $_ } | Select-Object -First 1
if ($primaryDriverLog) {
    $driverTail = Get-Content $primaryDriverLog -Tail 500 -ErrorAction SilentlyContinue
    $loads = $driverTail | Select-String 'OpenVR-SpaceOverride .+ loaded'
    $lastLoad = $loads | Select-Object -Last 1
    $ipcConnects = ($driverTail | Select-String 'IPC client connected').Count
    $ipcDisconnects = ($driverTail | Select-String 'IPC client disconnecting normally').Count
    $ipcErrors = $driverTail | Select-String 'CreateNamedPipe failed|ResponseError|disconnecting due to error'

    if ($lastLoad) {
        Write-Finding 'OK' "Driver: $($lastLoad.Line.Trim()) ($primaryDriverLog)"
    } else {
        Write-Finding 'WARN' "Driver log has no recent load line: $primaryDriverLog"
    }

    if ($ipcConnects -gt 3 -and $ipcDisconnects -gt 3) {
        Write-Finding 'WARN' "IPC churn in driver log tail: $ipcConnects connects / $ipcDisconnects disconnects (overlay restarts or SteamVR cycling)"
    } elseif ($ipcConnects -ge 1) {
        Write-Finding 'OK' "IPC sessions in tail: $ipcConnects connect(s), $ipcDisconnects clean disconnect(s)"
    }

    if ($ipcErrors) {
        Write-Finding 'WARN' 'Recent driver IPC errors:'
        $ipcErrors | Select-Object -Last 3 | ForEach-Object { Write-Host "       $($_.Line.Trim())" }
    }

    foreach ($stale in $DriverLogs | Select-Object -Skip 1) {
        if (-not (Test-Path $stale)) { continue }
        $staleTail = Get-Content $stale -Tail 30 -ErrorAction SilentlyContinue
        $staleVer = $staleTail | Select-String 'OpenVR-SpaceOverride .+ loaded' | Select-Object -Last 1
        if ($staleVer -and $staleVer -notmatch '8\.1\.0') {
            Write-Finding 'WARN' "Stale driver log ($stale): $($staleVer.Line.Trim()) — ignore; use %LOCALAPPDATA%\openvr\logs\"
        }
    }
} else {
    Write-Finding 'WARN' 'No driver log found — driver may not have loaded this boot'
}

Write-Host ''

# --- vrserver.txt ---
if (Test-Path $VrServerLog) {
    $vrTail = Get-Content $VrServerLog -Tail 800 -ErrorAction SilentlyContinue

    # Session start lines often fall outside the shutdown-heavy tail; search full log as fallback.
    $spaceLoad = $vrTail | Select-String 'Loaded server driver spaceoverride' | Select-Object -Last 1
    if (-not $spaceLoad) {
        $spaceLoad = Select-String -Path $VrServerLog -Pattern 'Loaded server driver spaceoverride' -ErrorAction SilentlyContinue |
            Select-Object -Last 1
    }
    if ($spaceLoad) {
        Write-Finding 'OK' "vrserver loaded spaceoverride: $($spaceLoad.Line.Trim())"
    } else {
        Write-Finding 'FAIL' 'vrserver never loaded spaceoverride — check driver manifest/DLL'
    }

    $standablePose = ($vrTail | Select-String 'PoseUpdated from driver standable').Count
    $standableCrash = $vrTail | Select-String 'Exception c0000005|Exception c0000409'
    if ($standablePose -gt 0) {
        Write-Finding 'WARN' "Standable FBE PoseUpdated spam: $standablePose lines in vrserver tail"
        Write-Host '       Standable sends poses for unowned device indices at startup/shutdown.'
        Write-Host '       Latest session ended with vrserver Exception c0000005 after Standable shutdown spam.'
        Write-Host '       Mitigation: quit Standable FBE before exiting SteamVR, or disable Standable driver when not in FBT.'
    }
    if ($standableCrash) {
        Write-Finding 'FAIL' "vrserver crash signature in log: $($standableCrash[-1].Line.Trim())"
        Write-Host '       This is usually Standable shutdown + vrserver teardown — not SpaceOverride pose hook.'
    }

    $overlayLaunch = $vrTail | Select-String 'Started auto launch dashboard nyabsi.spaceoverride' | Select-Object -Last 1
    if (-not $overlayLaunch) {
        $overlayLaunch = Select-String -Path $VrServerLog -Pattern 'Started auto launch dashboard nyabsi.spaceoverride' -ErrorAction SilentlyContinue |
            Select-Object -Last 1
    }
    if ($overlayLaunch) {
        Write-Finding 'OK' 'Space Override overlay autolaunch succeeded'
    } else {
        Write-Finding 'WARN' 'No overlay autolaunch line in vrserver log'
    }
} else {
    Write-Finding 'WARN' "Missing $VrServerLog"
}

Write-Host ''

# --- vrcompositor.txt ---
if (Test-Path $CompositorLog) {
    $compTail = Get-Content $CompositorLog -Tail 400 -ErrorAction SilentlyContinue
    $stats = $compTail | Select-String 'Total\.*presents\.*dropped\.*reprojected' | Select-Object -Last 3
    if ($stats) {
        Write-Host '--- Compositor (last app sessions) ---'
        $stats | ForEach-Object {
            $line = $_.Line.Trim()
            if ($line -match '(\d+) presents\.\s+(\d+) dropped\.\s+(\d+) reprojected') {
                $p, $d, $r = $Matches[1..3]
                $rpPct = if ([int]$p -gt 0) { [math]::Round(100.0 * [int]$r / [int]$p, 1) } else { 0 }
                $level = if ([int]$d -gt 0) { 'WARN' } elseif ($rpPct -gt 50) { 'WARN' } else { 'OK' }
                Write-Finding $level "$p frames, $d dropped, $r reprojected (${rpPct}% reprojection)"
            } else {
                Write-Host "       $line"
            }
        }
        Write-Host '       0 dropped is good. High reprojection often = SLAM/VD latency or fast head motion with Quest.'
    }
} else {
    Write-Finding 'WARN' "Missing $CompositorLog"
}

Write-Host ''

# --- steamvr.vrsettings ghosts ---
if (Test-Path $SteamVrSettings) {
    $settings = Get-Content $SteamVrSettings -Raw | ConvertFrom-Json
    if ($settings.PSObject.Properties.Name -contains 'driver_01spacecalibrator') {
        Write-Finding 'WARN' 'steamvr.vrsettings still has driver_01spacecalibrator (old Space Calibrator fork) — remove to avoid load-order confusion'
    }
    if ($settings.'steamvr'.debugInputBinding -eq $true) {
        Write-Finding 'WARN' 'debugInputBinding is on — adds SteamVR input noise'
    }
}

# --- Registry ---
try {
    $config = (Get-ItemProperty -Path $RegistryKey -Name Config -ErrorAction Stop).Config
    $prefs = (Get-ItemProperty -Path $RegistryKey -Name ConfigPrefs -ErrorAction SilentlyContinue).ConfigPrefs
    if ($config -and $config.Length -gt 20) {
        Write-Finding 'OK' "Calibration profile present ($($config.Length) chars)"
    } else {
        Write-Finding 'FAIL' 'Calibration profile empty — override inactive until calibrate'
    }
    if ($prefs -and $prefs.Length -gt 20) {
        Write-Finding 'OK' "User prefs present ($($prefs.Length) chars)"
    }
} catch {
    Write-Finding 'FAIL' 'No SpaceOverride registry profile'
}

Write-Host ''
Write-Host 'Run scripts/harden-steamvr-stack.ps1 to apply safe cleanups (SteamVR stopped).'