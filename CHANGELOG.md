# Changelog

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