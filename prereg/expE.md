---
created: 2026-11-02
experiment: Exp E (Mindfire plan item E7a)
status: pre-registered, committed before the first run of demo/exp_e.py
---

# Exp E — the kinematic simulator with the gait's measured tracking error

## Question

The pilot's arm difference appears under Isaac Sim physics (arm0 7/8 vs armB
2/8 controllers completing all four trips) but not in the kinematic simulator
(7/8 vs 6/8). The kinematic rollout executes every Twist exactly; the frozen
Go2 gait does not. **Is the gait's tracking error, injected as noise into the
kinematic rollout, enough to reproduce the physics arm difference?**

## Hypothesis

H_E: with AR(1) velocity noise fitted to the tracking error measured in the
Exp 3 physics traces, arm0 controllers complete all four trips (majority of 3
reps) more often than armB controllers.

## Design (fixed before running)

- **Controllers:** the 16 pilot controllers, `demo/arms/{arm0,armB}/k0..k7.py`,
  unmodified. The frozen base is a reference row and is in no test.
- **Simulator:** `demo/rollout.py` (unicycle, 50 Hz, 180 s budget, four legs,
  door shuts before leg 2), with its `noise` hook: the sampled residual is
  ADDED to the Twist that is integrated; the controller's observation and its
  own command are unchanged.
- **Noise model** (`demo/tracking.py`): per component (body-frame vx, vy, and
  wz), residual = realised − commanded velocity over each 50 Hz step, fitted as
  AR(1) with mean (mu, phi, sigma), lag pairs within episodes only, **pooled
  over every non-quarantined Exp 3 physics trace of every arm and the base**
  (arm-neutral). Fitted once by `python demo/exp_e.py --fit` after the Exp 3
  sweep completes; the fitted values go to `demo/out/v2/expE_noise_model.json`
  and are reported. No parameter is tuned after seeing E's outcome.
- **Reps:** 3 per controller. Rep r uses Exp 3's condition r (scenario seed r
  and the start offset in `demo/exp3.py`) and noise seed 5000 + r, so every
  controller gets the same noise stream in a given rep (paired).
- **Reference condition `none`:** the identical reps with no noise, reported
  beside, not tested.

## Primary metric and analysis

- **Success** = all four trips completed within 180 s.
- **Per controller:** success in at least 2 of its 3 reps (majority of 3).
- **Primary test:** Fisher exact, two-sided, arm0 vs armB on majority-of-3
  success, condition `noise`, n = 8 vs 8.
- **Pass criterion:** H_E is supported if arm0's count exceeds armB's AND
  p < 0.05. Anything else is reported as "not reproduced", with the counts.
- **Secondary (descriptive, no test):** per-episode success per arm; the same
  table for `none`; per-controller agreement of the `noise` majority verdict
  with the Exp 3 physics majority verdict (out of 16), against the same
  agreement for `none`.

## Declared limitations

- n = 8 per arm: the test has little power except for a gap as large as the
  pilot's. A null is not evidence that tracking error is irrelevant.
- The noise model is stationary: it ignores the dependence of the residual on
  the command (acceleration lag), on turning, and on contact. A pass says this
  coarse model is sufficient; a fail does not show the physics gap is not about
  tracking.
- Controllers with wall-clock planning budgets (`time.perf_counter`) behave
  differently under machine load; E runs alone on the machine, with no Isaac
  Sim running, and the load average is logged per rollout.
- These are the 16 pilot controllers that generated the hypothesis, so E is
  explanatory for the pilot, not a confirmatory test of the arm effect.
