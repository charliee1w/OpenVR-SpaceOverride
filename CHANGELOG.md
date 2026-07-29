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
