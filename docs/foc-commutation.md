# FOC commutation and alignment

Why the commutation angle is derived the way it is, and why the alignment sweep has the shape it does.
Source: `ClosedLoop::ComputeFocElectricalAngle()`, `ClosedLoop::GetFocCountsPerElecRev()`, and the
alignment sequence in `ClosedLoop::SetClosedLoopEnabled()`. Output stage in
`FocController::ApplyTorque()`.

Test rig: SAMME51 + SimpleFOCMini (DRV8313), later DRV8316 EVM; Lin Engineering BL17E19-01D with an
E5-1000 encoder, 24 V.

---

## <a name="three-symptoms"></a>The three symptoms this explains

Closed-loop BLDC mode originally had three faults that turned out to be two bugs:

1. It could not move at all in one direction from a fresh alignment — an "invisible wall".
2. It hit a wall at ~300 encoder counts in the other direction.
3. The wall's distance moved when `M569.1 C` changed.

Torque in this drive obeys `T ∝ q·cos(Δ)` where `Δ = θ_commanded − θ_rotor` is the commutation error.
Correct FOC holds `Δ ≡ 0`. Both bugs put terms into `Δ`.

---

## <a name="alignment-reference"></a>Why alignment targets 3π/2, not 0

`ApplyTorque()` issues a pure q-axis command, and the inverse Park transform that implements it
(`alpha = -q·sinθ, beta = q·cosθ`) places the stator vector **90 electrical degrees ahead** of the angle
passed in. That is correct for a torque command.

But the alignment sweep originally terminated at angle 0, so the rotor's d-axis parked at +90°, and the
runtime commutation formula then added the same +90° again. The vector applied at the alignment origin
landed exactly on the rotor d-axis: **zero torque at encoder count 0 regardless of commanded magnitude**,
and torque proportional to `sin(Δ)` rather than `cos(Δ)` either side of it.

`x = 0` became a zero-torque magnetic trap — and *stable* for one sign of torque, so the harder the loop
pushed the tighter it clamped. That is symptom 1 exactly.

Sweeping to `3π/2` (`alignTargetAngle = 3072`) cancels the built-in +90° so encoder count 0 is genuinely
electrical zero. SimpleFOC's `alignSensor()` uses `setPhaseVoltage(v, 0, _3PI_2)` for the same reason.

**Consequence worth keeping in mind:** because alignment settles the d-axis on electrical zero, a positive
`torqueMagnitude` is a genuine q-axis command at every encoder count *by construction*. That is why there
is no separate electrical-angle offset field and no q-axis verification step — SimpleFOC omits both for
the same reason. An offset field and a live-torque-response search for it both existed here at one point;
both were scaffolding for the bugs on this page, and the evidence that justified them was itself an
artefact of those bugs.

---

## <a name="encoder-direction"></a>Encoder direction is measured, not assumed

`focEncoderDirection` is the sign of `d(rotor electrical angle)/d(encoder count)`. The commutation formula
originally assumed +1. It is a property of the wiring — encoder A/B order versus motor phase order — and
on this rig it is inverted.

A sign inversion does not merely reverse the motor. It makes the commutation error grow at
*(assumed + true)* rate instead of cancelling, so the rotor accelerates away from the origin, passes peak
torque, and is trapped at the next zero-torque point a "false pole pitch" away. That is symptom 2, and its
distance is `2048 / (4096/countsPerElecRev_configured + |true slope|)` — which depends on `C`, hence
symptom 3.

Measured directly: the alignment sweep ramps the commanded angle 0 → 4095 while the raw encoder count runs
+300 → −840. Fitted against a C-sweep of 11 runs, predicted stall distances land within a few percent
across the whole range:

| C | predicted | measured |
|---|---|---|
| 800 | 277 | 277 |
| 1000 | 309 | 294 / 305 |
| 1500 | 365 | 377 |
| 2500 | 428 | 425 |
| 3500 | 461 | 457 |

**`focEncoderDirection` is applied twice** — to the commutation angle *and* to the torque sign — and both
are necessary. Correcting only the angle makes positive torque drive towards increasing rotor electrical
angle, which on a reversed encoder is the direction of *decreasing* count, leaving the position loop's
sign convention inverted relative to the torque it receives. That is positive feedback.

---

## <a name="sweep-structure"></a>Why the sweep has three phases

Settle at the start angle (120 ms) → sweep one electrical revolution (400 ms) → settle at the *same*
angle (180 ms). 700 ms total, well inside the ~900 ms CAN reply timeout that `M569 D4` must fit within.

The sweep itself (rather than a held fixed-angle pull) is required because a static pull is open-loop:
a detent between the rotor's unknown start position and the target can stop it short and zero it in the
wrong place. This showed up as repeated 40–90° shortfalls. A continuously-advancing field drags the rotor
past every detent at least once regardless of where it started.

The third phase is what makes the measurement possible. With **both endpoints settled at the same
commanded angle**, the signed count difference is a direct calibration: sign gives the direction,
magnitude gives counts per electrical revolution. Without it the start point is contaminated by the rotor
snapping in from wherever it powered up, which is why the older `preAlignCount` check could report only a
vague "plausible amount of motion" rather than a usable number.

The measurement is adopted only if within 0.5×–2× of the value derived from `C`/`L`. Outside that band it
falls back and raises a tuning error — a blocked or hand-held shaft would otherwise poison commutation
outright, which is worse than the assumption being replaced. The band is deliberately wide: its job is to
reject a *failed measurement*, not to police a genuine `C`/`L` misconfiguration, which is reported
separately so the drive still runs on the measured value.

---

## <a name="calibration-discrepancy"></a>Counts per electrical revolution

Measured 1136–1159 across runs, against 1133 derived from `C850 L3` — under 1% out.

This was not always so clean. With the earlier `C1000` configuration, three independent measurements (the
alignment sweep, the open-loop tracking slope, and the stall-distance fit) all put one electrical
revolution at ~1145 counts against the 1333 that `C1000 L3` implies. 4000/1145 = 3.49, not an integer
pole-pair count, so either the encoder was not 4000 counts/rev or the motor was not 3 pole pairs.

The 1141–1176 spread seen at one point was partly an artefact of the alignment sweep saturating the
current sense — see [foc-current-sense.md](foc-current-sense.md#alignment-overcurrent). With the current
limit in place the spread narrowed to 23 counts.

The drive prefers the measured value over the derived one in all cases, and `M569 D4` reports both so a
disagreement surfaces as a diagnostic rather than silently drifting the commutation angle.

---

## <a name="removed-limits"></a>Two limits that were removed

Both are recorded here because the source now carries only a short "deliberately not" note, and both are
the kind of thing that looks like an obvious missing feature to someone reading the loop fresh.

### No velocity ceiling on `vel_target`

One existed, borrowed from SimpleFOC's `P_angle.limit` and configured via `M569.1 Q`. It clamped the
**sum** of the position correction *and* the velocity/acceleration feedforward, making it a hard ceiling
on total commanded axis velocity rather than a limit on the correction.

Above that ceiling `vel_error` goes negative and the inner loop commands reverse torque, so the axis
simply cannot exceed `Q`: measured at `Q4000` against a move peaking at 53,000 counts/s — 13× over the
cap — the loop brakes at full authority. It also duplicated a limit the motion planner already enforces
(`M203`), in different units.

The reason it was added in the first place — "a stalled rotor only gets a strong demand after error has
built up" — was a symptom of the commutation bugs above, not of an unclamped `vel_target`.

If a genuine approach-rate limit is ever wanted, clamp `Kpp × currentPositionError` **alone** and leave
the feedforward terms outside the clamp, so it can never cap the achievable feedrate.

Note that SimpleFOC retains this clamp, and correctly so for its use case: it has no motion planner
supplying a trajectory, so clamping the setpoint is the only velocity limit available.

### No slew-rate limit on commanded torque

One existed, mirroring SimpleFOC's PID `output_ramp`, on the theory that it bounded phase current
`di/dt`. In a voltage-mode drive with no current loop it does not: peak current is set by the voltage
*magnitude*, which `focMaxTorque` and `GetFocVoltageScale()` already clamp, and the winding's own L/R time
constant (~0.9 ms here) low-passes current far more aggressively than any rate an ~80 µs control tick
could impose. A ramp slow enough to matter electrically is one that detunes the loop by an order of
magnitude.

This reasoning applies to *voltage* mode. With the current loops closed, a slew limit on the current-loop
output is a different proposition and is listed as a candidate in the SimpleFOC comparison.
