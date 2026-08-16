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

Consequences of the inversion: losing sight of the tracker is close to a non-event, tracker noise is filtered into a slowly-varying correction rather than displayed, and SLAM relocation steps — the moment the headset suddenly re-decides where it is — are detected against the tracker and the translation component is cancelled in the same frame, so the published pose stays put while SLAM jumps underneath it. Cancellation is deliberately conservative: it requires the head to be nearly still, applies to translation only (a yaw relocation is left to the estimator), and is rate-limited, so it suppresses the common case rather than every case.

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

These apply **live — no SteamVR restart and no re-calibration.** The overlay writes them through SteamVR's settings store and the driver re-reads them as it runs, so you can also set them by hand in `steamvr.vrsettings`. Shipped defaults are in `dev-resources/driver/resources/settings/default.vrsettings`, and both default to off, so the driver behaves like upstream unless you ask otherwise.

### Filters and bounds

Every filter on the pose path can be switched independently, in the same settings block and on the same live poll. There is no overlay control for these yet — edit `steamvr.vrsettings` by hand.

| Setting | Default | What it does |
| --- | --- | --- |
| `trackerFilter` | **off** | Kalman pre-filter on the tracker's position. In **override** mode this filtered position *is* your published head pose. |
| `driftFilter` | on | One-Euro smoothing of the SLAM↔lighthouse correction. Smooths the correction, not your head. |
| `headVelFilter` | on | One-Euro on the *published* angular velocity. Your pose is identical either way; only apps that extrapolate between frames feel it. |
| `publishSlew` | on | **Safety bound.** Approaches a far-away pose candidate instead of snapping to it. |
| `corrRateLimit` | on | **Safety bound.** Caps how fast the correction itself may move. |

`trackerFilter` defaults **off**, which is a change from earlier builds. With the shipped constants its gain settles to ~0.15, which is roughly **125 ms of lag on head position**, and its adaptive term only opens up past about **6 mm of movement per frame**. A normal nod moves a helmet-mounted tracker just under that threshold, so nodding took the full lag while large fast motions passed through — which feels like the headset resisting you rather than being smoothed. What it removes in exchange is well under a millimetre of noise. If you preferred the old behaviour, set it back to `true`; it applies immediately.

> [!WARNING]
> The bottom two are **limiters, not smoothing**, and neither adds any latency to real head motion. Turning off `publishSlew` lets the view teleport when the pose jumps. Turning off `corrRateLimit` restores single-frame view rotations that were measured at up to 17.9° — enough to be genuinely unpleasant. If something feels laggy, these are not the cause.

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
- **Cover ground early, and cover it in every direction.** This is the single biggest lever on calibration quality. The scale of your headset's tracking space can only be measured by *moving through* space, and its precision improves in direct proportion to how much ground you covered — head rotation alone cannot determine it at all. Walk to the far corners in the first part of the run, pausing at each. If your coverage is lopsided the coaching now names the direction it needs ("thin left-to-right", "thin front-to-back", "thin vertically") rather than just asking for more.
- **Slow beats fast, and samples are only taken while you are still.** Samples captured during fast head rotation *or* fast translation are discarded: a wireless headset's pose and a lighthouse tracker's pose arrive with different latency, so a pair captured mid-motion pairs two instants that are not the same instant, and that skew is baked into the offset. The motion that works is move -> pause -> look around -> repeat; the on-screen coaching says which one is currently blocking.
- **The bar filling is the floor, not the finish.** Once coverage is met the run keeps collecting for as long as you keep improving it, up to 20 more seconds, and the hint changes to "keep moving to sharpen the scale fit". Earlier builds solved the instant the bar filled, which reliably produced calibrations at the bare minimum coverage. Stopping when the bar fills now costs you accuracy.
- **Headset scale is averaged across calibrations.** It is the one calibrated quantity that cannot be re-estimated while you play, so a single unlucky run used to be frozen in for the whole session. Each new measurement is now blended into a running average over your last several calibrations, which reduced its run-to-run scatter about elevenfold on the rig this was measured on. The average resets by itself if you change headset or if a measurement moves by more than 3%, so a genuinely changed setup is still tracked. Practical consequence: **a few good calibrations beat one perfect one**, and one bad run no longer ruins your session.
- **A brief tracking dropout no longer ruins the run.** Losing sight of a base station for a moment costs a few samples; only a sustained loss of about two seconds aborts. Recovered dropouts are noted in the log.
- **A bad calibration is refused rather than saved.** If the residual comes out too high, calibration aborts and your previous profile is restored. Re-run at a slower **Calibration Speed**; a rushed pass is the most common cause of a bad result.
- **Edit Calibration** nudges the saved transform by hand; **Remove Calibration** clears it. Re-calibrating properly beats hand-editing in almost every case.

## Tested scope

This fork has been used on exactly one rig:

- Quest Pro over Virtual Desktop
- Lighthouse base stations — one until 2026-08-15, two since
- Vive Tracker 3.0 as the head tracker
- Index controllers and lighthouse body trackers, plus Tundra trackers

**Nothing else has been verified here** — including Pico, ALVR, Steam Link, Quest Link / Air Link, and wired or DisplayPort headsets. Upstream supports a considerably wider range and carries a compatibility table for it; that table is deliberately **not** reproduced here, because none of it was checked against this fork's changes. Untested does not mean broken — the override path is upstream's and should behave as it always has — but this fork cannot vouch for it, and fusion in particular has comparatively few real sessions behind it. Treat fusion as experimental.

A second base station was added on 2026-08-15 and measured before and after. At the timescales the estimator works over, it cut the tracker's correlated position error by about **42%**, and the improvement grows the longer the interval — the signature of a drift-like error being replaced by more frequent optical fixes. It did **not** change anything about the SLAM↔lighthouse relationship: the correction still starts each session displaced by half a metre or more and still wanders during play, because those are properties of the headset's own tracking, not of lighthouse coverage. **More base stations improve the tracker; they do not remove the need for this driver.**

## Logs and diagnostics

Written automatically to `%LOCALAPPDATA%\OpenVR-SpaceOverride\logs\`:

- `spaceoverride_driver.log` — rolling log across sessions
- `session_*.log` — one per SteamVR start; older ones are pruned automatically
- `fusion_diag_*.csv` — per-frame diagnostics, only while **Diagnostic log (CSV)** is on
- `calibration.log` — one line per calibration, successful **or** failed. Failed runs record `result=abort` with the reason and the coverage they reached, so a run that died leaves a trace instead of vanishing.
- `calsamples_*.jsonl` — the raw pose pairs a calibration was solved from, one file per solve (~30 KB). These make a calibration re-solvable offline, which is what allows a change to the solver to be measured rather than merely believed.

The calibration line is worth knowing how to read. `hmdScale` is the value that ships; `hmdscale_raw` is what this run alone measured, and `hmdscale_n` is how many runs are in the average. **`hmdscale_raw` jumping around while `hmdScale` stays put is the averaging working, not a fault.** `scale_src` says whether the run actually measured scale (`measured*`) or kept the previous value (`kept_*`), and `spread_m` is the coverage that determined which — below 0.15 m, scale cannot be measured at all.

Session logs record the active mode and mode switches, tracker good/bad transitions, gate and hold events, correction resets, periodic heartbeats, and an end-of-session summary. **If you report a problem, attach the session log** — it is far more useful than a description, and it is the authority on what the driver actually did on the build you are running.

The calibration profile is mirrored to `%LOCALAPPDATA%\OpenVR-SpaceOverride\profile-backup.json` each time it is saved, so a lost registry key is recoverable.

Leave the CSV off for normal use; it is only for investigating tracking quality.

## Troubleshooting

**My view is completely messed up.**
The headset lost or reset its own tracking space. Re-calibrate.

**Calibration feels subtly off.**
Re-run at a slower calibration speed, moving smoothly through varied head orientations — and cover more ground than feels necessary, especially early in the run. Misalignment that gets *worse the further you walk from where you calibrated* is a scale error specifically; the cure is more positional coverage, not more samples. Because scale is now averaged across runs, two or three good calibrations will pull it in even if one of them was mediocre.

**My head feels sluggish or "sticky", like something resists small movements — nodding especially.**
Check `trackerFilter` in `steamvr.vrsettings`. It pre-filters the tracker's position, and in override mode that filtered position *is* your head pose, adding roughly 125 ms of lag to movements below about 6 mm per frame — which is exactly where ordinary nodding falls, while larger motions pass through unaffected. It ships **off** since 2026-08-16; if you are on an older build or have set it to `true`, set it to `false`. The change applies immediately with no restart.

**Which mode am I actually in?**
The session log says, at startup and on every mode switch, and every heartbeat line carries `mode=F` (fusion) or `mode=O` (override). Note that if `fusionMode` is absent from `steamvr.vrsettings` entirely you get the shipped default, which is **override** — an absent key is not the same as "unchanged from last session". The `mode` column in `fusion_diag_*.csv` is **not** a mode indicator; that file only exists while fusion is running.

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
- **Publish gates** — finite-value checks, and bounded reconvergence so a far-away candidate pose is slewed toward rather than snapped to. The gate runs on the composed world pose, so it covers both the tracker-driven pose and the fusion correction, and applies on the SLAM-fallback path as well. Position and rotation are each rate-limited, with the slew budget taken from the publish interval rather than from elapsed time.
- **Brief tracker-loss handling** — the last good pose is held for a short window before falling back.
- **Head tracker quashing** so applications don't bind it as a body joint.
- **Automatic session logging** with pruning of old session files, plus the optional per-frame diagnostic CSV.
- **Calibration profile backup** alongside the registry entry.
- **Calibration sample quality gating** — rejection of samples taken during fast head rotation or translation, a spread requirement before headset scale is fitted at all, and live on-screen coaching for the motion the solver still needs, naming the direction coverage is weakest in.
- **Headset scale averaged across calibrations** — bounded running average with automatic reset on a headset change or a step too large to be noise. Scale is the only calibrated quantity the runtime cannot re-estimate, so a single bad run used to be frozen in for the session.
- **Coverage-driven finish** — calibration keeps sampling while coverage is still improving instead of solving the moment the minimum is met, because scale precision is proportional to how much ground was covered.
- **A solve on independent observations** — samples are collapsed to one average per distinct pose, so the estimator's independence assumption holds instead of being corrected for after the fact.
- **Tracking-loss tolerance during calibration** — a momentary dropout costs a few samples rather than the whole run.
- **A calibration record that includes failures**, plus raw sample dumps that make a calibration re-solvable offline.
- **Individually switchable filters and bounds** — every filter on the pose path can be turned off at runtime, including the tracker position pre-filter, which is now off by default.
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
| Shipped driver settings | `dev-resources/driver/resources/settings/default.vrsettings` |

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
