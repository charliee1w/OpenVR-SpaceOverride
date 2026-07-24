# OpenVR-SpaceOverride — fusion fork

**This is a modified version of [OpenVR-SpaceOverride](https://github.com/Nyabsi/OpenVR-SpaceOverride) by [Nyabsi](https://github.com/Nyabsi)**, which itself builds on [OpenVR Space Calibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator) by [pushrax](https://github.com/pushrax). This fork is maintained by **helpgore** at [charliee1w/OpenVR-SpaceOverride](https://github.com/charliee1w/OpenVR-SpaceOverride). See [What this fork changes](#what-this-fork-changes).

The problem is unchanged from upstream: a SLAM-tracked headset and your lighthouse-tracked devices live in two coordinate spaces that slowly slide apart, so a one-time playspace calibration decays and your head stops agreeing with your hands and feet. With a rigid tracker mounted to the headset, calibration solves the fixed transform between that tracker and your head, which gives the driver a continuously observable relationship between the two systems instead of a single stale offset.

What the driver *does* with that relationship is where this fork diverges: it ships two opposite modes.

> [!IMPORTANT]
> **Questions or problems with this build go to helpgore — Discord `helpgore`.**
> Please don't send them upstream: this tree contains substantial code upstream's author did not write, and behavior described here may not exist upstream at all, so the contact address in upstream's README is the wrong place for it. If you can reproduce a problem on an unmodified upstream build, that one belongs upstream — everything else belongs here.

## Requirements

- Lighthouse tracking (or equivalent), with at least one base station
- A rigid tracker mounted firmly to the headset (Vive Tracker 3.0 or equivalent)
- A headset with its own positional tracking

The mount matters more than anything else here. If the tracker can shift on the headset, the transform calibration measured is no longer true, and no amount of software makes up for it.

## Two modes

Both modes end in the same place — head and body in one aligned space. They differ in **which device is trusted to draw your view**, and neither is universally correct.

**Override** — upstream's behavior, and the default. After calibration the headset pose is *built from* the head-mounted tracker. Head and body then share a single tracking system, so there is nothing left to drift against. The cost: the tracker's tracking quality becomes your view's tracking quality. Its jitter is jitter in your view, and when it loses line of sight, that lands in your face.

**Fusion** — added by this fork, and inverted. The headset's own SLAM pose drives the view; the tracker is demoted to an *observer* whose only job is to continuously estimate the SLAM→lighthouse correction that keeps your view anchored in lighthouse space.

| | Override | Fusion |
| --- | --- | --- |
| You look through | the tracker's pose | the headset's own SLAM pose |
| The mounted tracker is | the pose source | an observer estimating the correction |
| Headset SLAM required | no | **yes — it is the pose source** |
| Tracker jitter | enters your view | absorbed into the estimate |
| Tracker loses line of sight | you feel it | correction freezes, SLAM carries |
| Head and body in one space | by construction | by continuous correction |

Consequences of the inversion: losing sight of the tracker is close to a non-event, tracker noise is filtered into a slowly-varying correction rather than displayed, and SLAM relocation steps — the moment the headset suddenly re-decides where it is — are detected against the tracker and cancelled in the same frame, so the published pose stays put while SLAM jumps underneath it.

The estimator is an error-state Kalman filter over a yaw + translation correction, with Mahalanobis innovation gating for outlier rejection and a covariance reset when disagreement persists. Measurement noise is inflated with motion, which is how tracker-vs-SLAM latency skew is absorbed rather than chased. See `FusionEkfUpdate` in `src/driver/ServerTrackedDeviceProvider.cpp`.

### Which one to use

**Fusion assumes the headset's SLAM is genuinely good.** It trades "your view is only as good as the tracker" for "your view is only as good as your SLAM." If your headset's own tracking is weak, drifty, or prone to relocating, fusion inherits that directly and **override is likely the better choice** — that is the situation upstream's design was built for, and it remains a legitimate answer.

Fusion is not a universal upgrade; it inverts the premise this project was originally built on, and it is the less-proven of the two paths here. If you are unsure, calibrate in override mode first, confirm everything works, then try fusion. They are one checkbox apart.

In either mode, while the override is active the head tracker's *published* pose is parked far from the play area so that applications such as VRChat and Standable do not bind it as a body joint. The device still enumerates, and the driver still uses its real pose internally.

## Switching modes

Two toggles at the top of the overlay's **Settings** tab:

| Overlay control | Setting | Default |
| --- | --- | --- |
| **Fusion mode** | `driver_spaceoverride/fusionMode` | off (override) |
| **Diagnostic log (CSV)** | `driver_spaceoverride/fusionDiag` | off |

These apply **live — no SteamVR restart and no re-calibration.** The overlay writes them through SteamVR's settings store and the driver re-reads them as it runs, so you can also set them by hand in `steamvr.vrsettings`. Shipped defaults are in `resources/settings/default.vrsettings`, and both default to off, so the driver behaves like upstream unless you ask otherwise.

Switching mid-session is safe by construction: whichever estimator takes over starts from a clean state and re-converges, and the publish gate bounds the transition, so the pose slews rather than snapping.

Your calibration is shared by both modes and does not need to be redone when you switch. The rest of the Settings tab still requires a re-calibration, as the tab itself notes.

> [!NOTE]
> **Discard Calibrated Offset takes precedence over fusion.** With that advanced option enabled, the driver runs the classic tracker-driven path regardless of `fusionMode`, because that option exists specifically to feed raw tracker data through.

## Calibration

1. Mount the tracker rigidly to the headset. Power on your base station(s) and tracker, and make sure the tracker is visible.
2. Press **Calibrate**. The on-screen text tells you what it wants at each stage — first **move your head around** so the driver can work out which tracker is on your head, then the guided pass: *look left, center, right, center, up, center*.
3. The profile saves itself and the override keeps running in the background.

Worth knowing:

- **Turn and tilt, don't just pan.** If every sample looks like the same rotation, calibration will say your movement is too uniform and keep collecting.
- **Slow beats fast.** Samples taken during fast head rotation are discarded outright: a wireless headset's pose and a lighthouse tracker's pose arrive with different latency, and pairing them mid-whip bakes that skew permanently into the offset. Moving smoothly means fewer rejected samples and a better fit.
- **A bad calibration is refused rather than saved.** If the residual comes out too high, calibration aborts and your previous profile is restored. Re-run at a slower **Calibration Speed**; a rushed pass is the most common cause of a bad result.
- **Edit Calibration** nudges the saved transform by hand; **Remove Calibration** clears it. Re-calibrating properly beats hand-editing in almost every case.

## Tested scope

This fork has been used on exactly one rig:

- Quest Pro over Virtual Desktop
- A single lighthouse base station
- Vive Tracker 3.0 as the head tracker
- Index controllers and lighthouse body trackers

**Nothing else has been verified here** — including Pico, ALVR, Steam Link, Quest Link / Air Link, wired and DisplayPort headsets, multi-base-station setups, and Tundra trackers. Upstream supports a considerably wider range and carries a compatibility table for it; that table is deliberately **not** reproduced here, because none of it was checked against this fork's changes. Untested does not mean broken — the override path is upstream's and should behave as it always has — but this fork cannot vouch for it, and fusion in particular has comparatively few real sessions behind it. Treat fusion as experimental.

## Logs and diagnostics

Written automatically to `%LOCALAPPDATA%\OpenVR-SpaceOverride\logs\`:

- `spaceoverride_driver.log` — rolling log across sessions
- `session_*.log` — one per SteamVR start; older ones are pruned automatically
- `fusion_diag_*.csv` — per-frame diagnostics, only while **Diagnostic log (CSV)** is on

Session logs record the active mode and mode switches, tracker good/bad transitions, gate and hold events, correction resets, periodic heartbeats, and an end-of-session summary. **If you report a problem, attach the session log** — it is far more useful than a description, and it is the authority on what the driver actually did on the build you are running.

The calibration profile is mirrored to `%LOCALAPPDATA%\OpenVR-SpaceOverride\profile-backup.json` each time it is saved, so a lost registry key is recoverable.

Leave the CSV off for normal use; it is only for investigating tracking quality.

## Troubleshooting

**My view is completely messed up.**
The headset lost or reset its own tracking space. Re-calibrate.

**Calibration feels subtly off.**
Re-run at a slower calibration speed, moving smoothly through varied head orientations.

**My head tracker looks like it flew far away in SteamVR.**
Deliberate — see [Two modes](#two-modes). Its published pose is parked so apps don't bind it as a body joint; the driver still uses the real pose.

**The head tracker keeps losing line of sight.**
In fusion mode this is largely harmless: the correction stops updating while the headset's own tracking carries the view, then resumes. In override mode you will feel it — brief losses hold the last good pose, then fall back to SLAM if **Fallback to SLAM** is on. If your tracker is occluded often, fusion is the mode designed for that situation.

**My controllers jump, then settle back.**
Expected. Controllers ride the headset's inside-out tracking, which drifts against lighthouse space; the driver measures that drift and eases them back.

**I'm stuck facing one direction.**
Don't run OVR Advanced Settings alongside it, then try again.

**My full-body tracking looks strange in VRChat.**
Known, and not introduced by this driver: precise head-and-body alignment exposes quirks in VRChat's IK that looser calibration methods hide. Every lighthouse-tracked HMD setup meets the same thing.

**Does it drift?**
Not by design. Drift that survives a good calibration almost always traces back to lighthouse conditions — occlusion, reflective surfaces, base station placement. Fix the tracking environment first.

## FAQ

**Does the driver stop using the headset's own pose?**
Depends on the mode, and this is the single most important thing to get right. **Override:** yes — the pose is built from the tracker. **Fusion:** no, the opposite — SLAM is the pose source and the tracker only observes the correction. Upstream's README describes the override behavior as the project's thesis; in this fork that is accurate for one of two modes.

**Can I disable the headset's own tracking, or run it in 3DoF?**
**Only in override mode**, where the pose is reconstructed from the tracker and SLAM is not required.

> [!WARNING]
> Do **not** do this in fusion mode. Fusion's whole design is that the headset's own tracking draws your view; turning it off leaves nothing driving the view at all. Upstream's README suggests disabling headset tracking as an option — that advice belongs to the override path only.

**How is this different from TrackingOverride?**
TrackingOverride substitutes one device's pose for another's and leaves the tracker-to-headset offset for you to solve. This solves that offset during calibration and applies it continuously. Fusion additionally uses it to *correct* SLAM rather than replace it.

**Does it conflict with OpenVR Space Calibrator?**
No. Both can be installed; they solve alignment differently, and Space Calibrator's presence doesn't break the override.

**What about jittery trackers, like Tundras, as the head tracker?**
Upstream's objection is sound *for override mode*: the mounted tracker draws your entire view, so jitter that is a nuisance on a foot is nauseating on your head, and a stable tracker is the right call. That rationale does not apply in fusion mode, where the tracker never draws the view — its noise feeds a correction estimate that is filtered and gated. It is therefore plausible that fusion makes jittery trackers usable as head trackers. **This is reasoning from the design, not a measurement:** no such hardware has been tested here, and a tracker with correlated or bursty error rather than plain noise could still degrade the estimate. If you try it, the diagnostic CSV will tell you more than your gut will.

**Can wireless latency affect the pose?**
In override mode, yes — the tracker and the headset's display pipeline run on different clocks, so streaming latency can show up as lag or swimming in a tracker-driven view. In fusion mode the published head pose is the headset's own native streamed pose, so head motion behaves exactly as it natively does on your setup; latency instead affects the correction estimate, where measurement noise is inflated with motion so skew is absorbed rather than chased. A solid connection is worth having either way.

## Advanced: Discard Calibrated Offset

> [!CAUTION]
> Don't enable this unless you already understand manual TrackingOverride setups. Most people have no reason to touch it.

This feeds raw tracker data with the calibrated offset applied straight through, making the headset behave as a native lighthouse device. It discards SLAM entirely — including as a fallback — and **bypasses fusion mode**. The trade-off is a yaw mismatch between the tracker's projection and the headset's, which is why it only works in local tracking space, where re-centering can correct yaw; stage space never re-centers yaw. After a headset restart the yaw alignment may need redoing.

To use it: enable the option, run calibration, then stand up straight and rotate your body while re-centering your *headset's* space (not SteamVR's) until the view lines up.

## What this fork changes

Relative to upstream, this fork adds:

- **Fusion mode** — an inverted pose path where the headset's SLAM drives the view and the mounted tracker only observes the SLAM→lighthouse correction, estimated by an error-state Kalman filter with innovation gating and covariance-reset re-anchoring, including detection and same-frame cancellation of SLAM relocation steps.
- **Live mode switching** from the overlay, without a SteamVR restart, with estimator state reset and a bounded transition on switch.
- **Publish gates** — finite-value checks, and bounded reconvergence so a far-away candidate pose is slewed toward rather than snapped to. The invariant is that the published head pose cannot teleport in a single frame, whatever the inputs do.
- **Brief tracker-loss handling** — the last good pose is held for a short window before falling back.
- **Head tracker quashing** so applications don't bind it as a body joint.
- **Automatic session logging** with pruning of old session files, plus the optional per-frame diagnostic CSV.
- **Calibration profile backup** alongside the registry entry.
- **Calibration sample quality gating** — rejection of samples taken during fast head rotation.
- **Cross-thread synchronization** between IPC configuration writes and pose callbacks, so a configuration change cannot tear an in-flight pose.
- **This README**, rewritten for the fork, correcting upstream statements that fusion mode invalidates.

## Source map

| Area | File |
| --- | --- |
| Mode behavior, publish gates, fusion estimator | `src/driver/ServerTrackedDeviceProvider.cpp` |
| Filters | `src/driver/KalmanFilter.cpp`, `include/driver/OneEuroFilter.h` |
| Log locations and pruning | `src/driver/Logging.cpp` |
| Overlay Settings tab | `src/overlay/UserInterface.cpp` |
| Calibration and sample gating | `src/overlay/Calibration.cpp` |
| Profile storage and backup | `src/overlay/Configuration.cpp` |
| Driver/overlay IPC | `include/shared/Protocol.h` |
| Shipped driver settings | `resources/settings/default.vrsettings` |

Specific thresholds, filter constants and gate limits are intentionally not restated here — read them from the source above, or from a session log, which always reflects the build you are actually running.

## Building

Windows, MSVC, CMake with the presets in `CMakePresets.json`. Clone with submodules; the dependencies under `3rdparty/` are required. The build produces the overlay executable and `driver_spaceoverride.dll`, and the driver folder layout SteamVR expects is under `dev-resources/`.

## Credits

- **helpgore** — this fork: fusion mode and its estimator, the pose gates, diagnostics and the rest of [what this fork changes](#what-this-fork-changes). Contact on Discord as `helpgore` for anything about this build.
- [Nyabsi](https://github.com/Nyabsi) — [OpenVR-SpaceOverride](https://github.com/Nyabsi/OpenVR-SpaceOverride), the upstream project this forks. The override pose path, the calibration solver, the overlay and the driver architecture are his work, and this fork inherits all of it.
- [pushrax](https://github.com/pushrax) — [OpenVR Space Calibrator](https://github.com/pushrax/OpenVR-SpaceCalibrator), substantial parts of which this lineage uses and descends from.

Everything listed in [What this fork changes](#what-this-fork-changes) is this fork's; everything else belongs to upstream.

## License

Commits up to and including `1cc0583` are MIT (see [`LICENSE.MIT`](LICENSE.MIT), Copyright Justin Li). Everything after is AGPL-3.0-only (see [`LICENSE`](LICENSE)).

This is a modified version of the upstream project; the modifications are summarised in [What this fork changes](#what-this-fork-changes). If you distribute this software or make it available to users over a network, the AGPL's terms — including the obligation to offer corresponding source — apply to you.
