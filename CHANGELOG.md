# Changelog

Changes in this fork (`vfinal-fusion`), maintained by **helpgore**, relative to
upstream [Nyabsi/OpenVR-SpaceOverride](https://github.com/Nyabsi/OpenVR-SpaceOverride).

The headline addition is **fusion mode**. Everything else is either hardening of the
pose pipeline, or making calibration produce a trustworthy result without demanding
unreasonable precision from the person doing it.

---

## Calibration accuracy: scale pooling, plateau finish, station-collapsed solve (2026-08-15)

Built after the second base station arrived (HW1) and a measurement pass over 84 logged
calibrations. `protocol::Version` is unchanged at 7 — nothing here crosses the IPC boundary.

**`hmdScale` is now averaged across calibrations.** Scale is the one calibrated quantity the
runtime can never re-estimate: fusion applies it directly and the EKF carries yaw + translation
with no scale state, so whatever a run draws is frozen until the next calibration. Measured over
14 consecutive solves it scattered **σ 0.757%, peak-to-peak 2.63%**, against a per-solve error
bar averaging 0.202% — the solves disagree **3.8× more than they claim to**, so no gate tuned
against that error bar can catch it. The remedy is the scheme already applied to the lever arm:
bounded running average (cap 8), owner-serial keyed so a headset change resets it, and a 3%
step reset (~4σ) so a genuinely changed setup is tracked rather than blended. Only runs that
actually *measured* scale are pooled — every `kept_*`/`default_*` path returns the prior, and
pooling one would blend the average into itself. Replayed over the real history
(`archive/n2a-capture-reader/hw1_scale_pool_replay.py`): shipped-value σ **0.757% → 0.068%**,
11.1×, which at 2 m from the origin is **15.1 mm → 1.4 mm** of permanent error. That is a
reduction in scatter about the pooled mean, not evidence the mean is unbiased.

**Runs no longer finish at the bare minimum.** Solving fired the instant all three coverage
gates cleared, which pinned 12 of the last 14 calibrations between 0.150 and 0.199 m of spread
against a 0.150 m gate — and scale precision goes as 1/spread. The run now keeps collecting
while coverage is still improving, finishing 4 s after the last meaningful gain, capped at 20 s
extra. Gates are unchanged and still authoritative; only the solve is delayed.

**The solve runs on one averaged observation per station.** Samples arrive in bursts while the
user holds still, so within a station they are near-duplicates with intra-cluster correlation
~1 — which `EstimateHmdSpaceScale` already knew and patched over by inflating the reported error
by √(n/K) after the fact (N3-c). Collapsing makes the independence assumption *true* instead of
corrected-for: within-station noise averages down, the error bar becomes meaningful by
construction, and the O(N²) pair loops shrink ~45×. **Shipped without offline validation, at the
user's explicit direction** — there was no calibration replay harness to validate against, which
is why this build adds one.

**Raw sample dumps.** Every solve writes `calsamples_*.jsonl` — the raw pose pairs, station
index and metadata. `Agents.md` calls the missing replay harness "the binding constraint on
estimator work" for the driver; the calibration solver had the identical gap and no equivalent.
Solver changes were previously inspectable but not measurable.

**A pairwise-baseline scale estimator, logged only.** `|Δref| / |Δtarget|` over long baselines
recovers scale with rotation and translation cancelling exactly, instead of competing with six
nuisance parameters in one SVD. Median over pairs ≥ 20 cm. Emitted as `scale_pair=` for
comparison against `hmdscale_raw=` over real runs; **not applied** — changing the estimator and
the observation set in the same build would make a bad outcome uninterpretable.

**A single bad frame no longer kills a run.** `CollectSample` aborted the whole calibration —
state cleared, every sample discarded — the first time either pose came back invalid, and logged
only to the in-VR message list. Brief line-of-sight losses are routine. It now coasts, aborting
only after 2 s of sustained loss, and reports the dropped-frame count.

**Failed calibrations are recorded.** `calibration.log` held successful solves only, so an
abandoned or aborted run left no trace at all — a user reporting repeated failures produced a log
showing nothing but clean successes. Aborts now write `result=abort reason=…` with the coverage
reached.

**Coaching names the deficient direction.** "Cover more of your play space" is not actionable
for someone who has already walked a line across the room. The hint now names the weakest room
axis when coverage is lopsided by 4× or more.

**Every filter and bound on the pose path is now individually switchable at runtime.** Five keys
under `driver_spaceoverride/` in `steamvr.vrsettings`, polled once a second, so all of them take
effect without a SteamVR restart. They are driven from one table (`kFilterToggles`) that both the
startup read and the live poll walk, so adding a filter later cannot wire up one and miss the
other. Defaults are deliberately not uniform:

| key | default | what it is |
|-----|---------|-----------|
| `trackerFilter` | **off** | `KalmanFilterXYZ` on tracker position — in override mode this *is* the published head pose |
| `driftFilter` | on | One-Euro on the correction transform; smooths the correction, not head motion |
| `headVelFilter` | on | One-Euro on published angular velocity only; the pose is identical either way |
| `publishSlew` | on | **Safety bound** — `GatePublish` jump hold and reconvergence slew |
| `corrRateLimit` | on | **Safety bound** — the N1-h correction rate limit |

The last two are limiters, not smoothing, and are called out as such in the code: turning off
`publishSlew` means a far-away candidate is snapped to rather than approached, and turning off
`corrRateLimit` restores the measured 17.87° single-frame view yaw step on covariance reset.
Neither adds latency to real head motion, so neither is a candidate for "it feels laggy". Toggling
any filter resets the affected filter state, so re-enabling one cannot step the pose by however
far the head moved while it was bypassed.

**Why `trackerFilter` defaults off.** `KalmanFilterXYZ` pre-filters the tracker
position, and in override mode that filtered position *is* the published head pose. With the
shipped constants the gain settles to K = 0.146 — **~125 ms of lag** at a 50 Hz pose rate — and
the adaptive term only opens past a **6.3 mm per-frame dead-band**. A 20° nod moves a helmet-top
tracker ~7.7 cm, ~3.8 mm per frame: under the dead-band, so ordinary nodding takes the full lag
while large motions punch through. User-reported as "something prevents me from nodding".
`AUDIT.md` finding 12 flagged this lag and deferred it; N4-e step (1) is "drop
`KalmanFilterXYZ`". HW1 then halved the noise it suppresses while leaving its cost unchanged.
Live-pollable, so the old behaviour is one toggle away without a restart.

## Audit hardening, the correction rate bound, and the estimator extraction (2026-08-07 → 2026-08-10)

Everything in this entry shipped as the matched pair deployed 2026-08-10 (driver `87115DA2…`,
overlay `F3878598…`) and was exercised over two clean 68-minute sessions. Deep detail lives in
the workspace `AUDIT.md` (findings 11–25) and `Agents.md` (N1-h row, cr10 list); this is the
summary of what changed and why.

**Estimator extraction.** The fusion/override decision path moved out of
`ServerTrackedDeviceProvider.cpp` into `pose_est::ProcessHmdFrame`
(`PoseEstimator.cpp`/`PoseMath.h`) — a pure function of its inputs: no SteamVR calls, no locks,
no `GetRaw`. This is what made the offline replay harness possible (the Python replay reproduces
the driver's own diagnostics to print quantisation). Reviewed line-by-line against the
pre-refactor source; faithful.

**Correction rate bound (N1-h).** A covariance reset used to re-anchor yaw and translation in a
single frame — measured up to a 17.9° view step in 27.5 ms, because one 3-DoF position sample
cannot separate yaw from translation and the publish gate only tests position. The EKF's
*correction channel* is now rate-bounded (`kMaxCorrYawRate` ≈ 15 °/s, `kMaxCorrTransRate`
0.5 m/s) — the right place because no real head motion passes through it. A `corr_slew=` counter
in the heartbeat records every engagement. Offline A/B over the captured burst: peak correction
rate 649 → 14.9 °/s, single-frame steps >1° five → zero; bit-identical output on a clean session.

**Full-codebase audit remediation.** The pass that read every hand-written line (`AUDIT.md`)
fixed twelve findings, the most important being: a `DriverPose_t` size mismatch now **passes
poses through unmodified and logs once** instead of silently dropping every pose in the system;
the pose-hook depth counter is exception-safe (a throw used to pin it and leave the driver
inert with no log line); detours catch everything and publish the original pose rather than
unwinding into `vrserver`; the IPC server length-checks reads, uses atomic thread flags, joins
on `joinable()`, and value-initialises pipe instances; driver-side IPC setters validate their
inputs and reject-and-log; One-Euro clamps are NaN-safe; `LeaveStandby` resets the estimators;
the capture sink is size-capped and the rolling log rolls over; `ParseProfile` type-checks
required keys instead of reading indeterminate values under `NDEBUG`, and treats the three
fork-added booleans as optional so an older-but-valid profile no longer parses as corrupt.

**The ten 2026-08-08 review fixes** (session `5dc11699`), including: slew/covariance honesty,
fall-through `headVel` inertness in fusion, `lever_serial` persisted so a tracker-swap reset
survives an abandoned run, the capture flush ordered under both mutexes, `Role_Sync` device
rows captured post-correction (pre-correction rows made offline head-vs-body subtraction report
the entire correction as "body drift"), the diag epoch decoupled from CSV open,
`ComposeWorldPose` unified, the R9 diagnostic gated on `fusionDiag`, and session-stats gated by
mode.

**Overlay pre-flight for `SetHmdTracker`.** The audit added driver-side rejection of malformed
IPC — but the protocol has no NACK, and the overlay's applied-state cache (the C8 anti-spam
dedupe) records a sent message as applied. One driver-side rejection therefore became
*permanent silent divergence*: the UI showed the new calibration while the driver kept the old
one, and the dedupe suppressed every retry for the life of the overlay process. The overlay now
mirrors the driver's acceptance rules and refuses to send (with a logged reason) anything the
driver would reject.

*Operational note, same evening:* **Continuous sync was enabled in the overlay UI** (profile
`continuousSync: true`) — the first activation of the pre-existing slam-sync path on this rig.
The overlay enrolls every non-lighthouse, non-reference tracked device (in practice: the
Standable virtual pucks) so body devices receive the live correction and no longer diverge from
the corrected view. No code changed for this; recorded here because session logs from 2026-08-10
onward show `SetSlamSync` enrollment lines that no earlier log contains.

## Review fixes, honest calibration error, and two diagnostics (2026-08-06)

**Pose path (driver).**

- A tracker swap could **permanently delete the head-tracker lever arm**. The offset was cleared at
  the *start* of sampling, but the profile stays valid, so any save before the run solved — overlay
  shutdown, or three UI toggles — wrote the cleared value to both the registry and the JSON mirror.
  The reset is now deferred until a calibration actually solves.
- An **empty serial read** (routine after a device wakes) was treated as a tracker swap, firing that
  wipe and storing an empty serial that broke target recovery.
- The tracker position filter was fed **unpredicted** samples on the fusion path and
  **velocity-predicted** samples on the override path. Fusion falls through to override when SLAM is
  briefly invalid, so a dropout flipped the filter's input convention frame to frame and injected a
  `velocity × prediction` step into filter state — which then arrived at the estimator as a
  measurement. Prediction and the One-Euro head filter are now inert on that fall-through, matching
  what fusion mode was already documented to do.
- The heartbeat reported estimator state whenever fusion mode was set, but the estimator only runs
  when fusion is set *and* native mode is off — so native mode printed an untouched filter as though
  it were wildly unconverged.
- The diagnostic CSV's time origin was stamped after the file opened, so the first rows after a
  mid-session re-enable carried the previous epoch.

**Calibration (overlay).** The reported scale standard error assumed every sample was independent.
Samples arrive in bursts while you hold still at a station, so the effective count is the number of
stations, not samples. The log now carries a cluster-corrected figure beside the original; measured
inflation is ~2.3×. The acceptance gate deliberately still reads the original value — it was tuned
against it — and moving it needs data, not a guess.

**Diagnostics (log-only, nothing reads them, pose path untouched).**

- Angular-velocity frame check: OpenVR documents the axis-angle encoding but never states the frame,
  and this driver assumed both conventions in different places. The check scores both against a
  reference differentiated from successive world rotations, counting only samples where the two
  actually disagree. **Result: device-local**, by 1.75× over ~84k discriminating samples — the
  existing tracker-cache read was right.
- The slam-sync device set is now logged. It never was, which made "my body drifts relative to my
  view" undiagnosable — a lighthouse device wrongly in that set gets the correction applied twice.
- The heartbeat logs total absorbed SLAM-origin motion beside the correction magnitude, so a large
  correction can be read as explained rather than alarming.
- Head-vs-body lighthouse poses moved into unified capture (`Rec_Device`); the short-lived
  `body_diag_*.csv` sink is gone.

## Capture schema rev2 — the file can now explain itself (2026-08-06)

Parsing the first real capture (43 min, 90.7 MB) showed the framing and the estimator-input
contract were sound — `Rec_Config` covers `PoseConfig` 12/12, `Rec_Frame` covers
`HmdInput`/`TrackerInput`, nothing truncated, no non-finite values — but that a reader still could
not answer basic questions about the file. `kFormatVersion` stays 1; this is append-only, so rev1
readers parse rev2 files and the rev2 reader parses the existing rev1 capture (both verified).

- **`Rec_Event` was never written.** `CaptureEvent` existed with zero call sites, so a capture had
  no events at all and could not explain its own 32 s frame gap. Now wired into `LogSession` — the
  one funnel every `LOG()` reaches — and placed *before* the session-log handle check, so the
  mirror cannot go silent because a different sink failed to open. That coupling is the same
  two-lifecycle mistake that shipped `body_diag` broken.
- **`Rec_DeviceInfo`** (id → serial, model, device class). `Rec_Device` carried only the OpenVR
  enumeration index, which is assigned in connection order and is not stable across restarts: the
  first capture held 16 devices that could not be told apart, not even a base station from a foot
  puck. Emitted once per device **per file**, keyed on a new `CaptureGeneration()` rather than a
  bool — re-opening the sink starts a new file, and that file has to describe its own devices.
- **`enabled` + `trackerID` appended to `Rec_Config`**, with `trackerID` added to the change test:
  it is not a `PoseConfig` field, so a tracker swap previously kept the old `configSeq` and a
  replay would have mis-attributed the new tracker's samples. `enabled` is 1 by construction there
  (the record is only emitted from the enabled path) — the disable *transition* is carried by the
  `Rec_Event` mirror, and the schema now says so instead of leaving it to be discovered.
- **`frameIndex` documented**: it counts records written, not hook calls, so its contiguity proves
  file integrity and *not* sampling completeness. The first capture was contiguous 0..84016 while
  averaging 32.6 Hz against a `displayHz` of 80.

Reader/validator: `archive/n2a-capture-reader/sorcap_check.py` (outside this repo).

## Unified capture (2026-08-06)

Machine-readable sink `capture_<ts>.sor` (format in `include/shared/CaptureFormat.h`) records the
**estimator inputs** — not derived metrics — so offline replay can compute disp/corr/sig/NIS/gate
without another driver release cycle.

- One lifecycle: `SetCaptureEnabled(desired)` only (no Open/Close pair half-wired to one call site).
- `Rec_Config` on change + capture re-open; `Rec_Frame` every HMD tick **before** `ProcessHmdFrame`;
  `Rec_Device` for head/sync/lh lighthouse poses pre-correction (B6); `Rec_Event` for correlation.
- Forward-compatible framing (`type`/`len`); append-only schema rules in the header.
- Human session log stays. `fusion_diag_*.csv` still dual-writes for now (transitional).
- Recording I/O for N2-a offline harness is now the capture file; a reader is still open.

## Offline pose path (N2-a Phase 1, 2026-08-05)

The HMD fusion/override decision path (EKF update, jump gate, last-good hold,
bounded reconvergence, override rebuild) lives in `pose_est::ProcessHmdFrame`
(`PoseEstimator.cpp` / `PoseMath.h`). The MinHook pose callback only gathers
inputs under locks and calls that function. Offline tools can call the same
entry with synthetic timestamps and recorded samples — no SteamVR runtime
required for the estimator itself. Live recording is `capture_*.sor`; an offline reader/replay
CLI is still open.

---

## Fusion mode

A second way to align a SLAM-tracked headset with lighthouse devices, selectable in
the overlay.

Classic **override** mode (upstream's design, still available) reconstructs the
headset pose from the head-mounted tracker: the tracker drives the view. That is
drift-free, but it means every tracker glitch or line-of-sight drop is a glitch in
your view, and the headset's own tracking quality is discarded.

**Fusion** inverts it. The headset's own SLAM drives the view, and the tracker only
*observes* a correction between SLAM space and lighthouse space. An error-state
Kalman filter estimates that correction (yaw plus translation) and applies it
smoothly.

Consequences:

- A tracker occlusion is no longer an event. The correction simply stops being
  updated while SLAM carries the view.
- SLAM position snaps (inside-out relocalisation) are detected and cancelled rather
  than passed through to the view.
- Innovation gating rejects tracker glitches instead of chasing them; sustained
  disagreement triggers a covariance reset that re-anchors deliberately.
- Measured in practice at ~2 mm typical SLAM-vs-tracker displacement over sessions
  of 300k+ frames.

Prediction, angular-velocity passthrough and the One-Euro head filter are inert in
fusion mode; the overlay says so rather than leaving dead controls looking live.

## Calibration

Calibration is the only place several quantities can be determined, so its failure
modes were the most expensive. In fusion mode the filter continuously corrects yaw
and translation, but **scale is fixed at calibration time and cannot be recovered at
runtime** — which made the bugs below much worse than they looked.

- **A weak calibration no longer destroys a good headset scale.** When positional
  spread was too low to observe scale, the solver returned a hard `1.0` and saved it
  over the previous measurement. Since fusion applies that scale directly and the
  filter carries no scale state, the result was a permanent ~0.4% mismatch that
  recalibrating could not fix — orientation and translation self-heal, scale does
  not. Inconclusive runs now keep the previous measurement.
- **Sampling gates relaxed to be reachable.** Samples were dropped above 0.25 m/s,
  slower than a shuffle, so walking the room — the exact motion that positional
  spread requires — collected nothing and the progress bar appeared to stall. The
  fastest route to the sample count was to stand still, which is what starves the
  scale fit. Now 0.35 m/s (a slow walk keeps sampling), 2.0 rad/s angular, and
  sample targets halved to 250/140/60.
- **Completion measures coverage, not count.** Standing in one spot produced
  hundreds of near-identical samples that advanced the counter without adding
  information. Calibration now solves as soon as rotation variety and positional
  spread are both adequate; when they are not, it collects to the full count exactly
  as before, so it can only ever finish earlier, never on worse data.
- **Low-spread fits are filtered on plausibility, not precision.** The risk at low
  spread is not a slightly noisy scale but the occasional wild outlier — a 0.160 m
  run once fitted a scale 3.4% off, which the formal standard error rated at 0.2%
  and passed. Between 0.15 and 0.20 m of spread, a fit is accepted only within 2% of
  unity.
- **The solve is levelled to gravity.** Both spaces are gravity-referenced —
  lighthouse off the base station's accelerometer, SLAM off the headset IMU — so the
  true relative rotation is a heading, and pitch/roll in the solve is error unless it
  exceeds each device's gravity accuracy. It does not: two calibrations 18 seconds
  apart disagreed by up to 1.68°, which no gravity reference can physically do, and
  Vive Tracker 3.0 static rotational accuracy is published at 0.460°. Tilt is never
  corrected at runtime and costs ~28 mm of foot-vs-head displacement per degree
  through a 1.6 m torso, so this was the largest frozen error in the system.
  Residuals fell from 3.8–5.7 mm to 2.8–4.1 mm — removing two fitted parameters
  improved the fit, which is what happens when they were absorbing noise.
- **Progress counts stations, not samples.** Sampling is capped at 20 Hz and only
  runs while nearly still, so standing in one place emitted ~20 near-identical
  samples a second — each advancing the counter, none adding information. A station
  is one occupied pose, capped at ten samples. Calibrations now finish on 59–113
  samples instead of 250–500.
- **Finish requires stations + positional spread + axis variance (N3-a, 2026-08-01).**
  Station count alone used to complete runs in ~20 s of micro-yaw, with
  `scale_src=kept_low_spread` and a frozen residual offset. The progress bar is the
  minimum of the three fractions; station radii were widened (8 cm / 18°) so tiny
  head wiggles cannot mint stations; default speed is Slow. Lever-arm sample count
  (`lever_n`) is persisted so the running average survives relaunches.
- **Live motion coaching** during sampling: tells you which motion the solver still
  needs (rotation variety, more of the play space) and when samples are being
  dropped for moving too fast, instead of a bar that stalls without explanation.
- **Degenerate solves fail closed.** A NaN residual used to pass the quality check
  and reach the pose pipeline.
- Empty-delta guard in the rotation solve; chaperone geometry length validated
  before use (a malformed profile could overflow the heap).

## Profile persistence

- **The newer store wins.** The profile is written to both the registry and a JSON
  mirror. The registry write reported failure only to stderr — invisible for a
  windowed app — while the mirror was written regardless, so a failed write left the
  two disagreeing and the next launch loaded the stale one. Observed in the wild
  with the two stores holding calibrations ~92° apart in yaw, which presents as a
  calibration reverting on its own. Profiles now carry a save timestamp and the most
  recently saved store is loaded, with the other used as fallback if it does not
  parse; disagreement is repaired on load.
- **Registry writes are verified** by reading the value back.
- The mirror is now actually read. It was previously written but never consulted, so
  it recovered nothing.
- **Remove Calibration** clears both stores. It previously invalidated the profile
  in memory and then declined to save it, leaving the old calibration in place.

## Pose pipeline reliability

- **The publish gate inspects the composed world pose.** It previously looked only
  at the local position field, so in fusion mode — where the correction lives in the
  world-from-driver transform — corrections went out unchecked.
- **Rate limiting actually holds.** The slew budget was taken from elapsed time,
  which saturates after a frame gap and licensed steps of nearly a metre. Elapsed
  time and slew budget are now separate, and rotation is capped as well as position.
- Jump gate floors its timestep so micro frame gaps cannot look like a teleport.
- Last-known-good hold on tracker loss, bounded by age.
- Head tracker is parked out of the way ("quashed") so applications do not bind it
  as a body tracker.
- **N1 fusion measurement model (2026-07-31).** Dead-reckoned tracker samples no longer
  enter the EKF as fresh centimetre-class measurements: Rt/Rθ inflate with coast age,
  rotation is integrated with angular velocity over the same age as position, and a bad
  sample invalidates the cache instead of leaving a stale coast for 150 ms. Fusion never
  falls through to the override/One-Euro path on a single bad SLAM frame (unified
  `ResetEstimators`). Published velocity is finite-checked and clamped; the zero
  quaternion is rejected; `DeviceTransform` starts as identity/scale 1; the head lever
  scales with the same slam scale as position. `eTrackingResult` is cached — OutOfRange
  is accepted with inflated R. Slam-step cancellation inflates Pt so the filter does not
  claim sub-millimetre confidence in a state that just teleported.
- Fixed a use-after-free draining the IPC pipe set on shutdown.
- Fixed a data race on the correction state between the estimator and the pose
  threads.
- Overlay startup no longer aborts if the dashboard thumbnail cannot be set.

## Diagnostics

- Per-session log written on every SteamVR start, with a 60-second heartbeat
  carrying frame counts, gate outcomes, correction magnitude and filter uncertainty.
- Optional per-frame fusion CSV (head motion, innovation, gain, correction, world
  position, gate outcome) for offline analysis.
- Rows are batched in memory and flushed in blocks rather than per row, so the pose
  thread is not doing file I/O every frame.
- Logs older than 14 days are pruned.
- The calibration log records the solve internals that matter: where the scale came
  from (`measured`, `measured_lowspread`, `kept_low_spread`, `kept_lowspread_outlier`
  …), its standard error, sample counts and rejections by cause, spatial coverage,
  the solved rotation, and the pitch/roll tilt away from the expected convention
  flip.

## Where the remaining error is

Measured over 1.89 M fusion frames across 20 sessions (tracker loss 0.030%, zero
last-good holds, zero non-finite drops). Foot tracker relative to the headset view,
RMS in millimetres:

| source | typical | class |
|---|---|---|
| Frozen tilt (true gravity disagreement + mount pitch/roll) | 0–29 | setup / unmeasured |
| Quest SLAM gravity error, 0.1–0.3° × 1.6 m | 3–8 | irreducible |
| Head-tracker attitude accuracy (0.46°, published) | 1.5 | irreducible |
| Body tracker lighthouse noise, single base station | 1.5–2.1 | setup |
| EKF steady-state head placement | 0.9–1.3 | irreducible given Q/R |
| Tracker filter lag in the fusion path | 2.0–2.8 | code |
| Tracker↔SLAM time skew (~6 ms, measured) | 0.2 (7.0 at 1.2 m/s) | partly code |
| Residual `hmdScale` error 0.22% | 0.05 (1.2 at 1.2 m/s) | code, marginal |

Total excluding tilt: **~6.7 mm**. Addressing every code-fixable item perfectly gives
~6.1 mm — a difference of about 0.4 mm, which is not perceptible.

Two results worth recording because they close recurring questions:

- **The EKF matches theory to two significant figures.** Predicted steady state
  √Pt = 0.092 cm; logged `sig` across 745 heartbeats is 0.09–0.13 cm. Nothing in the
  filter is mis-modelled, and retuning `Qt`/`Rt0`/`kv`/`kw` is worth ~0.16 mm.
- **The wireless link is not the bottleneck.** Regressing per-frame innovation against
  head speed over 26,295 samples gives an effective tracker-vs-SLAM time skew of
  4–7 ms, not the 35–45 ms motion-to-photon latency of the stream. Timestamping and
  prediction are working correctly.

The calibration's absolute yaw cancels from the user-visible error: it appears in both
the body placement and the published head orientation. Calibrated yaw has been observed
to differ by ~18° between sessions with no effect on alignment — that is the headset's
SLAM origin heading changing, not a calibration fault. `corr_yaw` in the heartbeat
carries that offset plus accumulated SLAM drift and is a health indicator, not an
accuracy score.

## Notes

- Time-offset (τ) estimation was implemented, evaluated against real session data,
  and **removed**. At the ~2 mm tracker noise floor it had no observable signal: the
  post-fit residual is noise and τ·velocity is degenerate with the translation
  state, so the estimate wandered instead of converging. The skew is absorbed into
  motion-inflated measurement noise instead.
- A low RMS residual does not mean an accurate calibration. It is computed over a
  local sample cluster after six nuisance parameters are refitted, so it absorbs
  rotation error by construction. This is why the calibration log reports coverage
  and scale provenance rather than leaning on the residual. It is a NaN backstop and
  nothing more; it was briefly tightened to 12 mm and that was a mistake.
- Rotations logged as Euler triples are easy to misread. `(roll≈0, yaw≈172,
  pitch≈0.6)` and `(roll≈180, yaw≈8.7, pitch≈179)` are the same physical rotation;
  reading them as ~163° apart produced a phantom "bad solve" that cost two rounds of
  defensive complexity. Compare rotation matrices, not angle triples.
- Diagnostics that discard the quantity they report are worse than no diagnostic. A
  tilt metric returning `acos(|cos|)` is positively biased and can never average to
  zero, so it would confirm a real tilt regardless of the truth; and computing tilt
  after the solve has been levelled logs 0.00 forever. Both shipped here, and both
  destroyed measurements before being caught.
