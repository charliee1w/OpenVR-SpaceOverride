# OpenVR-SpaceOverride (community fork)

> **Fork notice:** Community-maintained fork of [Nyabsi's OpenVR-SpaceOverride](https://github.com/Nyabsi/OpenVR-SpaceOverride), with stability fixes, weighted calibration, ecosystem coexistence tooling, and improved deploy/diagnostics. **Windows only.** Report issues on [GitHub Issues](https://github.com/charliee1w/OpenVR-SpaceOverride/issues).

OpenVR-SpaceOverride aligns SLAM-tracked headsets (Quest via Virtual Desktop, Pico, ALVR, etc.) with lighthouse-tracked devices. After calibration, a **rigid-mounted Vive tracker** drives the HMD pose instead of SLAM — eliminating drift between tracking systems.

## Requirements

- **Windows 10/11** with SteamVR
- Lighthouse (or equivalent) base stations + Vive trackers
- **Rigid head-mounted tracker** (Vive Tracker 3.0 recommended; Tundra supported via **Tundra mode**)
- SLAM HMD (Quest/VD, Pico Connect, ALVR, wired SLAM headsets, etc.)
- Visual Studio 2022 build tools (only if building from source)

## Quick install (pre-built package)

1. Download the latest **Release** zip from [Releases](https://github.com/charliee1w/OpenVR-SpaceOverride/releases), or build locally with `scripts\build-release.ps1`.
2. Copy `dist\steam-driver\spaceoverride\` into:
   ```
   Steam\steamapps\common\SteamVR\drivers\spaceoverride
   ```
3. Start SteamVR once.
4. From `dist\OpenVR-SpaceOverride\`:
   ```powershell
   .\OpenVR-SpaceOverride.exe -installmanifest
   .\OpenVR-SpaceOverride.exe -activatemultipledrivers
   ```
5. Restart SteamVR. Open **Space Override** from the dashboard.
6. Press **Calibrate** and follow on-screen prompts.

Verify: **Settings → Diagnostics** should show `Pose hooks: installed`, `Driver override: active`, `Override: active`.

Optional stack cleanup after install or SteamVR updates:

```powershell
.\scripts\harden-steamvr-stack.ps1
```

## Build from source

```powershell
git clone https://github.com/charliee1w/OpenVR-SpaceOverride.git
cd OpenVR-SpaceOverride
.\scripts\build-release.ps1
```

Or manually:

```powershell
cmake -B build-overlay -DBUILD_OVERLAY=ON -DBUILD_DRIVER=ON -DBUILD_TESTS=ON
cmake --build build-overlay --config Release
ctest --test-dir build-overlay -C Release --output-on-failure
.\scripts\deploy-steamvr.ps1
```

`deploy-steamvr.ps1` copies the driver to Steam's driver folder (what `vrserver` loads) and refreshes `dist\`. Re-run as **Administrator** to also update `C:\Program Files\OpenVR-SpaceOverride\` for overlay autolaunch.

| Artifact | Load path |
|----------|-----------|
| Driver DLL | `SteamVR\drivers\spaceoverride\bin\win64\driver_spaceoverride.dll` |
| Overlay EXE | `C:\Program Files\OpenVR-SpaceOverride\` (autolaunch manifest) or portable `dist\OpenVR-SpaceOverride\` |

## First calibration

1. Mount a tracker rigidly on your headset.
2. Open Space Override → **Calibrate**.
3. Move your head when prompted (identifies the head tracker if multiple trackers exist).
4. Follow look left/center/right/up prompts.
5. Profile saves automatically to the Windows registry (`HKCU\...\OpenVR-SpaceOverride`).

Use **Slow** or **Very Slow** speed if RMS quality is poor. Check **Setup checklist** on the Calibration tab if override stays inactive.

## Recommended settings (v8.2+)

New profiles default to a low-latency, low-smoothing baseline. Tune in **Settings** after your first good calibration:

| Setting | Default | Notes |
|---------|---------|-------|
| Smooth headset tracker | Off | Turn on only for noisy trackers (e.g. Tundra — use **Tundra mode** instead when possible) |
| SLAM drift sync | Off | Enables continuous SLAM↔lighthouse correction for body trackers; opt in when needed |
| Prediction | 0 frames | Increase or enable auto-tune if wireless stream feels laggy |
| Quit Standable when SteamVR exits | On | Prevents `vrserver` crashes from Standable FBE shutdown spam |

Conservative VD/Quest preset (registry only, keeps your calibration):

```powershell
.\scripts\apply-safe-profile.ps1
```

Use **Re-apply driver settings** in Settings if Diagnostics shows UI/driver mismatch after toggling filters.

## Diagnostics (Settings tab)

| Field | Healthy value |
|-------|----------------|
| Pose hooks | installed |
| Driver override | active |
| Override | active |
| Runtime mount residual | < 30 mm when standing still |
| IPC | healthy |
| Head smoothing (driver) | matches your Settings toggle |
| SLAM drift sync (driver) | matches **SLAM drift sync** when enabled |

**Profile export/import** is under Diagnostics — useful for backups or moving between PCs.

## Scripts

| Script | Purpose |
|--------|---------|
| `build-release.ps1` | Release build, tests, portable `dist\` package |
| `deploy-steamvr.ps1` | Copy driver/overlay to install paths, register manifest |
| `harden-steamvr-stack.ps1` | Fix stale manifests, vrpathreg, appconfig; run smoke test |
| `smoke-test-steamvr.ps1` | Hash/freshness checks without a VR session |
| `validate-install.ps1` | Smoke test + `debug-steamvr.ps1` report |
| `debug-steamvr.ps1` | Detailed install and artifact status |
| `analyze-vr-logs.ps1` | Post-session log audit (`vrserver`, compositor, driver) |
| `apply-safe-profile.ps1` | Apply conservative registry prefs (VD/Quest) |
| `prune-driver-log.ps1` | Remove legacy multi-GB driver logs |
| `remove-stale-install.ps1` | Remove duplicate/stale install paths |
| `enable-spaceoverride-driver.ps1` / `disable-spaceoverride-driver.ps1` | Toggle driver without uninstalling |

## Troubleshooting

```powershell
.\scripts\validate-install.ps1
.\scripts\analyze-vr-logs.ps1      # after a bad session or crash
```

### Override inactive / no profile

- Registry profile empty → run Calibrate or Import a backup JSON.
- Tracker not found → ensure head tracker is on and serial matches saved profile.
- Stale overlay → re-run `deploy-steamvr.ps1` or `harden-steamvr-stack.ps1`.

### Head movement feels sluggish or can't nod quickly

- Disable **Smooth headset tracker** (default off in v8.2+).
- Set prediction to **0** unless you need wireless latency compensation.

### Calibration feels off

Re-calibrate with **Slow** speed. Check runtime residual in Diagnostics.

### Controllers jump then settle

Expected when **SLAM drift sync** is on — SLAM controllers are drift-corrected toward lighthouse space.

### Quest + Virtual Desktop guardian shifts

Diagnostics may show **Guardian shift suspect** when SLAM jumps but the tracker stays stable. Re-calibrate if playspace drifts.

### SteamVR crashes on exit with Standable FBE

Standable can spam `PoseUpdated` during shutdown and take down `vrserver`. Keep **Quit Standable when SteamVR exits** enabled (default), or quit Standable manually before closing SteamVR.

### Driver log huge

Old builds could spam logs. Current driver rotates logs at 50 MB under `%LOCALAPPDATA%\openvr\logs\`. Safe to delete legacy `SteamVR\bin\win64\space_calibrator_driver.log` if multi-GB.

## Compatibility

| Streamer | Status |
|----------|--------|
| Virtual Desktop | Works |
| ALVR | Works |
| PICO Connect | Works |
| Steam Link | Works |
| Wired SLAM (Pimax, PSVR2, etc.) | Works |
| Meta Air Link | Unconfirmed |

Works alongside **OpenVR Space Calibrator** — different approach, no conflict.

## Coexistence notes

| Component | Notes |
|-----------|-------|
| **Standable FBE** | Virtual body trackers are excluded from SLAM drift sync. Auto-quit on SteamVR exit is recommended. Optional: apply calibration transform to Standable trackers. |
| **SmoothTracking** | Independent pose pipeline; no known conflict. |
| **SteamVR updates** | Can break vtable hooks — check driver log for `Enabled hook for IVRServerDriverHost` after updates; re-run `harden-steamvr-stack.ps1`. |

## Acknowledgements

- [Nyabsi](https://github.com/Nyabsi) — original OpenVR-SpaceOverride
- [pushrax](https://github.com/pushrax) — [OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) (calibration math heritage)

## License

Commits up to and including `1cc0583` are MIT ([`LICENSE.MIT`](LICENSE.MIT)). Later commits are AGPLv3 ([`LICENSE`](LICENSE)).

See [`CHANGELOG.md`](CHANGELOG.md) for fork-specific changes.