# Changelog

## 8.1.0 — wireless feel pass (2026-06-29)

### Logging
- Driver log moved to `%LOCALAPPDATA%\openvr\logs\` (no more 10 GB files in `SteamVR\bin\win64`)
- Runtime rotation at 50 MB (not only on driver load)
- Rate-limited IPC connect/disconnect/pipe-busy log spam
- `scripts/prune-driver-log.ps1` to remove legacy oversized logs

### Feel / prediction
- Prediction auto-tune now allows up to 4 frames with a wireless stream bias (+0.75 frames)
- Fallback prediction when auto-tune is warming up: 2 frames
- First-time calibration enables prediction auto and angular velocity by default
- Profile migration: missing `predictionAuto` / `eAngVel` keys default to on

### Mount maintenance
- Auto partial recal when runtime mount residual stays > 30 mm for ~7.5 s (5 min cooldown)
- Settings checkbox to disable auto partial recal
- Runtime residual streak resets after successful full or partial calibration

## 8.0.0 — stranger-ready fork release (2026-06-29)

### Install & docs
- Portable package layout (`dist/steam-driver`, `dist/OpenVR-SpaceOverride`)
- `build-release.ps1`, `package-portable.ps1`, `validate-install.ps1`
- README rewrite with step-by-step install and diagnostics
- Deploy registers overlay from portable path (no admin required)

### UX
- Setup checklist on Calibration tab when override inactive
- Driver telemetry fetched on startup
- Version shown as community fork

### Stability (7.x → 8.0)
- Protocol v9: IPC errors, driver telemetry, override inactive reasons
- Profile persistence fix (no wipe on quit; `SaveProfile` guards empty)
- Head tracker visual alignment in SteamVR playspace view
- Logging null-safety, shutdown pass-through, 50 MB log rotation
- Weighted calibration solver, runtime residual meter, guardian-shift warning
- Mount rigidity warning across partial recalibrations
- Profile export/import in Diagnostics

### Tests
- 22 Catch2 unit/integration tests (CI on Windows)