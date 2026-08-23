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

## <a name="counts-per-elec-rev"></a>Counts per electrical revolution

This number is a **scale factor on the commutation angle**, so any error in it accumulates with distance
travelled instead of averaging out. That property is what makes it worth this much attention.

### The failure that established it

Two runs, identical gains (`R20000 I0 D0 V1 A1 J0.0001`), differing only in what the alignment sweep
happened to measure:

| | measured | used | result |
|---|---|---|---|
| good | 1320 | 1320 | completes 12,508 steps, cruises at 0.75 A |
| bad | 1293 | 1293 | **stalls dead at 9,750 steps** at full current |

`C1000 L3` is 4000 counts per mechanical revolution over 3 pole pairs — **1333.33 exactly**, since both
are integers. Every measurement taken on the rig came in below it: 1293, 1294, 1301, 1320.

Working the bad run forward: the drive advances the angle 360° per 1293 counts, the rotor advances 360°
per 1333.33, so at the 9,750-count stall point the field is
`9750/1293 × 360 − 9750/1333.3 × 360 = 81°` ahead of where it should be. Torque follows `Iq·cos(Δ)`, and
`cos(81°) = 0.16`.

The log agrees precisely. Same speed, same load, current climbing the whole way:

```
t =  60 ms   err  −35    Iq −0.53 A     0.53 A is ample here
t = 340 ms   err  −73    Iq −1.14 A
t = 420 ms   err −106    Iq −1.77 A
t = 460 ms   err −171    Iq −1.99 A     current limit reached
t = 620 ms   err −2758   Iq −2.00 A     stopped, full current, no motion
```

Rising current at constant speed **is** the signature of a drifting commutation angle. Nothing else
produces it.

> **`Id ≈ 0` does not mean commutation is healthy.** It stayed at 0.03–0.05 A in both runs. The forward
> Park that measures and the inverse Park that drives use the *same* angle, so `Id` reads zero whether or
> not that angle matches the rotor — the frame is self-consistent while being 81° off the real q-axis.
> Torque per amp is the only quantity that reveals it.

### Why it took so long to appear

The error is proportional to distance, so a short move hides it:

| move | counts | Δ at 1293 | torque penalty |
|---|---|---|---|
| 50 mm | 6,250 | 52° | 1.6× current — survivable |
| 100 mm | 12,500 | 105° | past 90°, torque reverses |

Every 50 mm test in this project ran with this error present. Doubling the move length is what finally
pushed it past what the loop could compensate for.

### What the sweep is for

It is the only way to learn the encoder-to-electrical **direction**, and it is a good **check** that `C`
and `L` describe the motor actually connected. It is a poor way to *measure* the period: a 400 ms
mechanical experiment where the rotor settles short of the commanded angle by its load angle, biased low
every single time.

So the policy is now:

- **Direction** always comes from the sweep.
- **Period** comes from `C`/`L` when the two agree to within 10% — exact integers beat a biased
  measurement.
- **Period** comes from the sweep only when they disagree by more than 10%, because then `C` or `L` does
  not describe this motor and a biased measurement is the better of two bad options. Reported as a
  warning.
- Outside 0.5×–2× the measurement is rejected outright and `TuningError::TooLittleMotion` /
  `TooMuchMotion` gates the drive — that band exists to catch a blocked or unpowered shaft, not to police
  configuration.

### Exact rational arithmetic

Even the right integer is not good enough. `1333.33` rounded to `1333` is a 0.025% scale error, which
still integrates:

| period used | worst angle error over 400,000 counts |
|---|---|
| `1293` (the bad run) | 105° **at 12,500 counts** |
| `1333` (rounded exact) | 27° |
| **`4000/3` (exact ratio)** | **0.087°** |

0.087° is one LSB of the 4096-step angle representation — the floor, and it does not accumulate.

`GetFocElecPeriod()` therefore returns the period as a numerator and denominator rather than a single
count, and `ComputeFocElectricalAngle()` reduces modulo the **numerator** — a whole number of electrical
revolutions, across which the angle repeats exactly — before scaling:

```cpp
remainder = (direction × count) mod numerator          // 0 .. numerator-1
angle     = remainder × denominator × 4096 / numerator // 64-bit intermediate
```

The 64-bit intermediate is not optional at the top of the range: a 20-bit absolute encoder with 8 pole
pairs needs `2^20 × 8 × 4096`, comfortably past `2^32`. The divide costs a few tens of cycles against an
80 µs tick.

When the period came from the sweep the denominator is 1, and the expression reduces to exactly what the
old code computed — verified identical across ±50,000 counts.

### Historical note

An earlier `C850 L3` configuration measured 1136–1159 against 1133 derived — under 1% out, and at the
time that looked like agreement worth trusting. It was, but only because those were short moves. The
1141–1176 spread seen before the alignment current limit went in was partly the sweep saturating the
current sense (see [foc-current-sense.md](foc-current-sense.md#alignment-overcurrent)).

There is still an open question from the `C1000` era: three independent methods (the alignment sweep, the
open-loop tracking slope, and the stall-distance fit) put one electrical revolution near 1145 counts, and
`4000/1145 = 3.49` is not an integer pole-pair count. If that measurement was real then either the
encoder is not 4000 counts/rev or the motor is not 3 pole pairs — and the 10% agreement band above would
route around it silently. Worth resolving on the bench rather than in firmware.
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
