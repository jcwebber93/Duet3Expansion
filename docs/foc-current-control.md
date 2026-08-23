# FOC current-mode (d/q) control

The innermost loop of the cascade: how it is structured, why the limits and anti-windup are shaped the
way they are, and how to tune it. Source: the current-mode branch of `ClosedLoop::ControlMotorCurrents()`,
`ClosedLoop::FocCurrentModeActive()`, `FocController::ApplyDqVoltage()`, `FocController::MeasureDq()`.

Configured by `M569.1 F{Kp, Ki, maxAmps}`. `F0:0:0` returns to voltage mode, which is also the default.

---

## <a name="what-changes"></a>What current mode changes

In voltage mode the velocity PID's output *is* the q-axis voltage. In current mode it becomes a q-axis
**current** demand, and two PI loops work out what voltage delivers it — one regulating `Iq` to the
demand, one regulating `Id` to zero.

`Id` is current on the direct axis: it produces no torque, only heat. Driving it to zero is most of the
point. In voltage mode it is whatever the winding impedance and any commutation error happen to leave
there.

### Measured result

Current off the q-axis, at matched electrical speeds:

| elec speed (rad/s) | voltage mode | current mode |
|---|---|---|
| 0–15 | +5.3° | **−0.6°** |
| 15–60 | +5.0° | **−1.1°** |
| 120–250 | +4.8° | **−1.1°** |
| 250–999 | **+24.4°** | **−3.2°** |

The speed-dependent impedance lag is gone, cancelled by the loop driving `Vd`. Peak phase current on a
fast move fell from about 4 A to 1.0 A for the same motion — the same work done with far less current,
because none is being wasted on the d-axis.

---

## <a name="preconditions"></a>Why it refuses to run

`FocCurrentModeActive()` requires gains set, alignment done, conversions arriving, zero offsets
calibrated, *and* the sense frame measured with confidence ≥ 0.5.

Every one of those has failed in practice during development, and a current loop closed through any of
them regulates confidently onto the wrong axis — worse than not closing one at all. `M122` reports
`RUNNING` or `configured but INACTIVE` so that a failed precondition cannot present as a tuning problem.

`FocMeasurementUsable()` is deliberately separate: `FocCurrentModeActive()` asks whether current mode is
configured and calibrated at all, that one asks whether the reading *in hand right now* is trustworthy.
Keeping them apart lets `M122` distinguish "never started" from "started and dropped out".

---

## <a name="limits"></a>Limiting and anti-windup

### The output limit is circular, not per-axis

Clipping `Vd` and `Vq` independently shortens one more than the other and so **rotates** the applied
vector away from where the loops asked for it. At saturation — exactly when the drive is working hardest
— that swings torque onto the direct axis. Scaling both by the same factor shortens the vector without
turning it.

SimpleFOC clips per-axis (`_constrain(voltage.d, ±voltage_limit)` and likewise for q); this is one of the
few places our implementation is ahead.

### Anti-windup is by back-calculation

On saturation the integrators are scaled by the same factor as the output, leaving them holding what
*would* have produced the voltage actually applied. Without it they accumulate against an error the
hardware cannot answer, and unwinding takes as long as it took to build.

Integrators are also cleared when a move starts — they hold the previous move's operating point, and
unwinding that is a torque transient at the worst possible moment — and whenever the gains change.

### `K` clamps demand, not current

`K` is the ceiling on the *demand*. The loops may command up to `maxOutput` regardless, and `maxOutput`
is a **voltage**: 0.25 of a 24 V bus into a 0.67 Ω winding is over 13 A. Hence the separate measured-
current trip at 1.5 × K, above the demand limit but inside the sense range.

Note that `1.5 × K` must stay inside what the hardware can measure. The sense rails at about ±6 A, so
above **K ≈ 3.5 A** the trip lands outside the measurable range and stops protecting anything.

Exceeding the trip does **not** leave current mode — the loops are what reduce the current, so removing
them is backwards. It squeezes the output ceiling to a quarter instead. See
[What happens when the measurement fails](#fallback).

---

## <a name="motor-parameters"></a>Motor parameters

**Datasheets quote terminal (line-to-line) values. The dq frame works in per-phase quantities** — a
factor of 2 for a wye winding. Getting this wrong by hand is easy and was got wrong once here.

For the BL17E19-01D:

| | terminal (spec) | per-phase | independently measured |
|---|---|---|---|
| R | 1.34 Ω | **0.67 Ω** | 0.62 Ω from the alignment ramp (8% out) |
| L | 1.15 mH | **0.575 mH** | 0.51 mH back-solved from working gains (11% out) |
| L/R | 0.9 ms quoted | 0.858 ms computed | consistent |

Back-EMF 4.16 V/kRPM line-to-line, so ~2.08 V/kRPM per phase.

> A resistance of 0.44 Ω was quoted at one point from `Vq = 0.87 V` at `Iq = 2.00 A`. **That figure is
> wrong.** It came from a standstill-stiffness log in which the shaft was being pushed by hand (measured
> velocity −63 to +156), so back-EMF was assisting the drive and the loop needed less voltage than R alone
> would demand. The alignment-ramp figure is the trustworthy one: the rotor is locked to the field, so
> there is no back-EMF.

---

## <a name="tuning"></a>Tuning, and why detuning is the wrong instinct

A current loop of bandwidth ω rad/s on a bus of V volts wants `Kp = L·ω/V` and `Ki = R·ω/V`. This is the
same formula SimpleFOC's `tuneCurrentController()` uses, and its default bandwidth is 300 Hz.

A PI current controller also wants **its zero on the plant pole**: `Ki/Kp = R/L`. For this motor that is
1165 /s.

Working values: **`F0.04:47:2.0`**. `Kp = 0.04` puts crossover at `Kp·Vdc/L` ≈ 266 Hz.

### Reducing this gain is destabilising, not stabilising

`Kp = 0.04` gives a proportional loop gain near 1.0, which looks aggressive in isolation. **It is not,
and detuning it caused a runaway to 7.1 A** with the measurement frozen for 232 consecutive samples.

The mechanism is the cascade, not the loop:

```
slow current loop -> cannot deliver the demanded Iq -> motor falls behind ->
velocity loop winds up demanding more -> current loop commands maximum voltage ->
6V into 0.45 ohm -> saturation -> latch
```

An inner loop's job is to be fast enough that the outer loop sees it as instantaneous. Slowing it breaks
the timescale separation the cascade depends on. **Raise this gain if the response is sluggish; lower it
only for genuine ringing.**

Keep bandwidth well under 1/150 µs regardless — the three phase readings span 150 µs (see
[foc-current-sense.md](foc-current-sense.md#dseq-scan)) and a loop faster than its own measurement chases
sampling artefacts.

---

## <a name="o-parameter"></a>What `O` (voltage limit) now does

Its role changed when the current loop closed, and it is worth being explicit because the parameter looks
unchanged.

**Before:** `O` was the current limit by proxy. With no current feedback `I = V/R`, so the voltage ceiling
*was* the current ceiling. Unlimited, 24 V into 0.67 Ω is 36 A.

**Now:** `K` is a direct current limit, which is strictly better. But `O` cannot be relaxed, because the
voltage-mode fallback still exists and in that state `O` is the only bound again:

| O | fallback worst case | vs the 16 A driver trip |
|---|---|---|
| 6 V | 9 A | ok |
| 10 V | 15 A | marginal |
| 12 V | 18 A | **trips** |

**And `O` now costs speed.** At 2 A the IR drop is 1.34 V, leaving 4.66 V for back-EMF, so `O6` runs out
at roughly **2200 RPM**. Raising `K` for stiffness lowers that further — at 3.5 A the ceiling drops to
about 1750 RPM. `K` and `O` trade against each other.

The two roles now genuinely conflict: current mode wants `O` large for headroom, the fallback wants it
small. Splitting them — a tighter ceiling applying only when current mode is inactive — would resolve it,
and would also close the stall gap below.

---

## <a name="fallback"></a>What happens when the measurement fails

This is the most consequential piece of the current-mode path, and the first version of it was wrong in a
way that made things worse rather than merely not better.

### The original behaviour, and why it latched

The mode gate was a single condition:

```cpp
if (FocCurrentModeActive() && FocMeasurementUsable() && currentWithinLimit) { /* current mode */ }
else                                                                        { /* voltage mode */ }
```

Voltage mode clamps the position loop's output to `maxOutput` = `W × O/N`. On the test rig that is 0.375
of a 24 V bus — **9 V into a 1.34 Ω terminal resistance, or 6.7 A.** So losing the measurement did not
reduce the drive's authority, it *removed the only thing regulating it*.

That closes a loop with no exit:

```
current briefly exceeds the ±6 A sense range
   -> amplifiers clip -> measurement unusable
      -> leave current mode -> position loop gets full voltage authority
         -> 9 V applied -> 6.7 A -> still past the sense range
            -> measurement still unusable ------------------------------┐
                                                                        │
      <-----------------------------------------------------------------┘
```

Observed on the rig: **164 consecutive control ticks (656 ms)** at 7.1 A phase current with no way back,
and in a separate run the drive was already latched *before the move started* and had to be stopped with
an emergency stop. `M122` showed `sat=4719 ok=436` — over 90% of scans saturated — with
`Isense A=7.042A` and `Vq=9.00V`, which is `maxOutput × Vbus` exactly.

The trigger in that run was not even an overcurrent. The measured velocity glitched by 100 mm/s for one
tick; at `R20000` that is a 335-unit swing in a ±256 control signal, the output slammed to the ceiling,
and the latch did the rest.

### Three separate defects

**1. An overcurrent report was treated as missing data.** A saturated scan is the sense telling you the
current is past full scale. It was folded into the staleness count and became indistinguishable from "no
scan arrived". Fixed in `FocCurrentSense` — see
[foc-current-sense.md](foc-current-sense.md#saturation).

**2. The trip dropped out of current mode.** `currentWithinLimit` sat in the mode gate, so exceeding the
trip *removed the current loops*. But the loops are the mechanism that brings current down — they respond
to a high `Iq` by commanding less voltage. Dropping them at that moment is precisely backwards. The trip
now stays inside current mode and squeezes the output ceiling to `OverTripOutputFraction` (25%) instead,
which forces the same loops to back off.

**3. The fallback had full authority.** Fixed below.

### The bounded, decaying fallback

The `else` branch now distinguishes two cases that had been sharing one path:

| case | ceiling |
|---|---|
| `M569.1 F` absent or zeroed — **voltage mode by configuration** | `maxOutput`, unchanged |
| `F` set but the measurement is unusable — **a fault** | `focFallbackCeiling`, decaying |

`focFallbackCeiling` is re-seeded on every successful current-mode tick from the output the loops are
*actually* using:

```cpp
focFallbackCeiling = min(appliedMagnitude × FallbackHeadroom, maxOutput);
```

That value has a useful property: it was, one tick ago, producing a regulated current at or below
`focMaxCurrent`. Resuming from it is a bumpless transfer into a level already known to be safe, rather
than a jump to full authority.

While the measurement stays unusable the ceiling decays:

| condition | time constant | rationale |
|---|---|---|
| stale scan | 100 ms | a brief gap barely dents the output; motion is not disturbed |
| **saturated** | **10 ms** | already past the sense range — collapse |

Below 2% of `maxOutput` it snaps to zero, so a persistent fault ends with the drive quiet.

**The decay is also the recovery path.** As the ceiling falls the current falls with it, comes back inside
the sense range, the amplifiers stop clipping, `saturatedScans` clears on the next accepted set, and the
loops resume — which immediately re-seeds the ceiling. Nothing needs to detect "the fault is over"; it
falls out of the same mechanism that contains it.

### What this does not do

It does not stop the motor. A drive that has decayed to a zero ceiling produces no torque, so position
error grows and the existing stall detection (`M569.1 E`) fires `Heat::NewDriverFault()` in the normal
way. That is the intended escalation — the fallback's job is to stop the drive hurting itself while the
existing fault path decides what to do about the move.

It also means a rig whose current sense never works at all, but which *is* configured for current mode,
will not move. That is deliberate: the alternative is the 7 A behaviour above. `M122` says so directly —
`FOC current mode: LIMITED (sense saturated)` with `fallback ceiling=0.0000 of 0.3750`.

### Reading it in `M122`

```
FOC current mode: RUNNING, Kp=0.0400 Ki=47.0 maxI=2.00A, IqTarget=0.172A, integ d=-0.000 q=0.002
FOC limits: dropouts=2 overcurrent-ticks=0, fallback ceiling=0.0143 of 0.3750
```

- `RUNNING` / `LIMITED (stale measurement)` / `LIMITED (sense saturated)` / `configured but INACTIVE`
- `dropouts` counts transitions out of current mode; `overcurrent-ticks` counts control ticks spent over
  the trip while still regulating.
- A `fallback ceiling` well below `maxOutput` during a `LIMITED` state is the containment working. If it
  ever sits at `maxOutput` while `LIMITED`, something has bypassed the seeding.


---

## <a name="known-gap"></a>Known gap: voltage mode has no current protection

The [bounded fallback](#fallback) covers the case where current mode is *configured*. A stall there now
resolves the way it should: the stalled motor draws its way past the sense range, saturation is detected,
the ceiling collapses on the 10 ms constant, and the position error that is no longer being corrected
trips `M569.1 E` stall detection into `Heat::NewDriverFault()`.

**The gap is the other configuration.** With `F` absent or zeroed the drive is in voltage mode by choice,
keeps full `W × O/N` authority, and there is no measured-current trip at all — the `1.5 × K` check only
exists inside the current-mode branch. On a DRV8316 build that means current *visibility* with no current
*protection* beyond the driver's own 16 A OCP, which is set for the device rather than for the motor.

On the test rig `maxOutput` at `O9` is 9 V into 1.34 Ω — **6.7 A**, about 2.8× the motor's peak rating.
Lowering `O` is the only lever a user currently has, and it trades away running headroom to get it.

Closing this properly means applying a measured-current limit in voltage mode too, which is a
straightforward extension of the same ceiling mechanism but has not been done.
