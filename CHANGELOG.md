# Changelog

Changes in this fork (`vfinal-fusion`), maintained by **helpgore**, relative to
upstream [Nyabsi/OpenVR-SpaceOverride](https://github.com/Nyabsi/OpenVR-SpaceOverride).

The headline addition is **fusion mode**. Everything else is either hardening of the
pose pipeline, or making calibration produce a trustworthy result without demanding
unreasonable precision from the person doing it.

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

## Notes

- Time-offset (τ) estimation was implemented, evaluated against real session data,
  and **removed**. At the ~2 mm tracker noise floor it had no observable signal: the
  post-fit residual is noise and τ·velocity is degenerate with the translation
  state, so the estimate wandered instead of converging. The skew is absorbed into
  motion-inflated measurement noise instead.
- A low RMS residual does not mean an accurate calibration. It is computed over a
  local sample cluster, where a scale or rotation bias contributes only millimetres.
  This is why the calibration log reports coverage and scale provenance rather than
  leaning on the residual.
