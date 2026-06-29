# OpenVR-SpaceOverride (community fork)

> **Fork notice:** This is a community-maintained fork of [Nyabsi's OpenVR-SpaceOverride](https://github.com/Nyabsi/OpenVR-SpaceOverride), with stability fixes, diagnostics, weighted calibration, and improved deploy tooling. **Windows only.** Report issues on this repo's GitHub Issues.

OpenVR-SpaceOverride aligns SLAM-tracked headsets (Quest via Virtual Desktop, Pico, ALVR, etc.) with lighthouse-tracked devices. After calibration, a **rigid-mounted Vive tracker** drives the HMD pose instead of SLAM — eliminating drift between tracking systems.

## Requirements

- **Windows 10/11** with SteamVR
- Lighthouse (or equivalent) base stations + Vive trackers
- **Rigid head-mounted tracker** (Vive Tracker 3.0 recommended; Tundra supported with stronger smoothing)
- SLAM HMD (Quest/VD, Pico Connect, ALVR, wired SLAM headsets, etc.)
- Visual Studio 2022 build tools (only if building from source)

## Quick install (pre-built package)

1. Download the latest **Release** zip (or run `scripts\build-release.ps1` locally).
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

## Build from source

```powershell
git clone <your-repo-url>
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

`deploy-steamvr.ps1` copies the driver to Steam's driver folder and updates `dist\`. If Program Files copy fails, the portable `dist\OpenVR-SpaceOverride\` path still works for autolaunch (no admin).

## First calibration

1. Mount a tracker rigidly on your headset.
2. Open Space Override → **Calibrate**.
3. Move your head when prompted (identifies the head tracker if multiple trackers exist).
4. Follow look left/center/right/up prompts.
5. Profile saves automatically to the Windows registry.

Use **Slow** or **Very Slow** speed if RMS quality is poor. Check **Setup checklist** on the Calibration tab if override stays inactive.

## Diagnostics (Settings tab)

| Field | Healthy value |
|-------|----------------|
| Pose hooks | installed |
| Driver override | active |
| Override | active |
| Runtime mount residual | &lt; 30 mm when standing still |
| IPC | healthy |

**Profile export/import** is under Diagnostics — useful for backups or moving between PCs.

## Troubleshooting scripts

```powershell
.\scripts\validate-install.ps1   # smoke + debug report
.\scripts\debug-steamvr.ps1        # detailed status
```

### Override inactive / no profile

- Registry profile empty → run Calibrate or Import a backup JSON.
- Tracker not found → ensure head tracker is on and serial matches saved profile.
- Stale overlay → re-run `deploy-steamvr.ps1` or register manifest from `dist\OpenVR-SpaceOverride\`.

### Calibration feels off

Re-calibrate with **Slow** speed. Check runtime residual in Diagnostics.

### Controllers jump then settle

Expected — SLAM controllers are drift-corrected toward lighthouse space.

### Quest + Virtual Desktop guardian shifts

Diagnostics may show **Guardian shift suspect** when SLAM jumps but the tracker stays stable. Re-calibrate if playspace drifts.

### Driver log huge

Old builds could spam logs. Current driver rotates logs at 50 MB. Safe to delete `SteamVR\bin\win64\space_calibrator_driver.log` if multi-GB.

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

- **Standable FBE** and other pose-hook drivers: use with care; shutdown ordering can affect SteamVR stability.
- **SteamVR updates** can break vtable hooks — check driver log for `Enabled hook for IVRServerDriverHost` after updates.

## Acknowledgements

- [Nyabsi](https://github.com/Nyabsi) — original OpenVR-SpaceOverride
- [pushrax](https://github.com/pushrax) — [OpenVR-SpaceCalibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) (calibration math heritage)

## License

Commits up to and including `1cc0583` are MIT ([`LICENSE.MIT`](LICENSE.MIT)). Later commits are AGPLv3 ([`LICENSE`](LICENSE)).

See [`CHANGELOG.md`](CHANGELOG.md) for fork-specific changes.