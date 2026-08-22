# Closed-loop control structure

How `ClosedLoop::InstanceControlLoop()` came to be shaped the way it is, the control law it runs, and the
SVPWM output stage. Source: `ClosedLoop::InstanceControlLoop()`, `ClosedLoop::UpdateStallDetection()`,
`ClosedLoop::ControlMotorCurrents()`, `FocController::Svpwm()`.

---

## <a name="loop-restructure"></a>The three-branch restructure

`InstanceControlLoop()` had three parallel branches — DC servo, FOC, classic stepper — each with its own
copy of the control call, sample collection and statistics, **and** each falling through into a shared
tail that did the same things again. Three consequences, all silent:

- **DC servo executed `ControlMotorCurrents()` twice per tick** with an identical timestamp
  (`tuningError` is forced to 0 for DC servo, so the tail's guard passed). The I term integrated at
  double rate; the D term was attenuated to ~47% and its filter ran at twice the intended bandwidth,
  because `last_vel_error` was overwritten by the first call so the second saw `rawD = 0`.
- **FOC had no stall detection at all.** It lived only in the tail behind `tuningError == 0`, and a
  quadrature encoder permanently carries `NeedsBasicTuning`. The `!stall` guard in the FOC branch tested
  a flag that could never be set, and `M569.1 E` thresholds were inert.
- Statistics were double-counted for both, so `M122`'s average current fraction read roughly half.

Now: **per-drive-type preparation → one shared control call → one stall check → one sample → one
statistics update**, with the stall logic extracted into `UpdateStallDetection()`.

### Why the control law is not gated on `!stall`

For DC servo and FOC, `currentPositionError` is only computed *inside* `ControlMotorCurrents()`. Skipping
that call while stalled would freeze the error and latch the stall permanently. The stop action fires on
the detection edge instead, and `Heat::NewDriverFault()` is what actually stops the move.

A classic stepper is still left energised on stall — its phase currents are its holding torque — while DC
servo and FOC drop output.

---

## <a name="single-loop-pid"></a>The control law: single-loop position PID

All three drive types — DC servo, FOC and classic stepper — run the same five-term law in
`ControlMotorCurrents()`. Every term acts on **position error**, and the result is a torque demand in an
arbitrary -256..+256 range that each branch maps onto its own output stage.

| term | gain | `M569.1` | acts on |
|---|---|---|---|
| `PIDPTerm` | `Kp` | `R` | position error, clamped ±256 |
| `PIDITerm` | `Ki` | `I` | position error × time, clamped ±`PIDIlimit` (80) |
| `PIDDTerm` | `Kd` | `D` | `errorDerivativeFilter` derivative, clamped ±256 |
| `PIDVTerm` | `Kv` | `V` | commanded speed (feedforward) |
| `PIDATerm` | `Ka` | `A` | commanded acceleration (feedforward) |

`PIDJTerm` / `Kpp` (`M569.1 J`) is **not used**. The parameter still parses and reports, and the
telemetry channel still exists, but the law writes zero to it.

### Why the velocity loop was removed

An intermediate version ran a cascade instead: an outer position-P (`Kpp`) producing a velocity setpoint,
and an inner velocity PID where `Kp`/`Ki`/`Kd` acted on *velocity* error. For FOC this stacked on top of
the d/q current loops, giving three nested loops.

It worked, and on the BLDC rig it measurably outperformed the single loop on tracking. It was removed
because it could not be tuned reliably:

- **Three loops need timescale separation.** Two independently "working" tunes were found with nothing in
  common — `R1.05 I0.6 D0.001 J5.51` and `R250.05 I0 D2.11 J0.005` — which is the signature of a search
  space with no useful gradient.
- **Intuition inverts.** Lowering an inner-loop gain, the normal response to instability, *caused* a
  runaway: the inner loop stopped keeping up, the outer loop wound up, and the drive saturated. This is
  documented at [foc-current-control.md](foc-current-control.md#reducing-this-gain-is-destabilising-not-stabilising)
  for the current loop and applied equally to the velocity loop.
- **Every gain changed meaning** relative to a decade of Duet3D field tuning for `M569.1 R`/`I`/`D`.

The FOC path is still a cascade in the sense that this position PID feeds the d/q current loops. What
went is the *velocity* stage between them.

### Retuning after the revert

`Kp`/`Ki`/`Kd` now act on position error in steps, not velocity error. **Any cascade-era tune is
invalid.** Start with `R` alone (`I0 D0 V0 A0`), raise it until the shaft is stiff enough or ringing
starts, then add `I` for standstill holding and `D` last. `Kv`/`Ka` keep their upstream meaning.

### Filter feeding

`errorDerivativeFilter` supplies `PIDDTerm` and must be fed on every path that runs the law. The classic
stepper path feeds it in `InstanceControlLoop()`; the DC servo and FOC paths feed it inside their own
branches of `ControlMotorCurrents()`, because those branches compute `currentPositionError` themselves.
It is reset alongside `PIDITerm` at every move start and stall transition — `SetTargetToCurrentPosition()`
steps the error discontinuously, and stale history in a D term is a torque spike.

`speedFilter` is still fed everywhere, but only feeds telemetry (`CL_RECORD_MEASURED_VELOCITY`), the
stepper phase feedforward, and torque mode's speed limit.

---

## <a name="dangling-if"></a>A braceless `if` across a preprocessor boundary

Worth recording because the compiler could not warn about it. The DC-output-mode condition was written as
a braceless `if` on the line directly above the `if (seenT)` block, which made that block its **body**:

```cpp
if (seenU && (DcServoOutputMode)tempDcOutputMode != dcOutputMode)   // no braces
#endif
if (seenT)
{ ... }
```

With `SUPPORT_DCSERVO` enabled, the driver was therefore only dropped to open loop when `U` was *also*
supplied and changed. A plain `M569.1 P... T... C...` deleted and reallocated the encoder underneath a
still-running closed-loop control task, and swapped the commutation scale factor live with no
re-alignment.

`-Wmisleading-indentation` does not catch this: the `#endif` between the two lines breaks the
heuristic. Both conditions now share one properly braced statement.

---

## <a name="svpwm"></a>SVPWM: why the sector-based form

`FocController::Svpwm()` computes the sector and in-sector angle directly from `atan2f`/`sinf`.

A previous version derived the sector from the sign pattern of three 120°-separated projections and
switched on that pattern directly. That was **wrong in two independent ways**, both verified numerically
by a full-revolution sweep at 0.02° resolution checking duty-cycle continuity:

1. The sign pattern does not enumerate sectors in angular order — it visits them 1, 3, 2, 6, 4, 5 — so
   switching on it directly applied the wrong sector's formula most of the time.
2. Even after correcting the mis-ordering, the six per-sector `(t1, t2)` formulas were not mutually
   consistent at their shared boundaries; sector 2 and sector 3 disagreed by a large amount at the same
   physical angle.

Together these caused duty-cycle jumps of up to ~100% of the full PWM range at six fixed electrical
angles every revolution, independent of commanded torque magnitude or direction. That is sufficient to
explain high current draw with no clean torque-producing rotation, and intermittent driver overcurrent
faults specifically during sustained rotation.

The replacement was confirmed continuous (residual ~1e-4, i.e. floating-point step noise only) across a
full revolution at multiple torque magnitudes.

**Do not revert to a sign-pattern sector lookup.**

### A cheaper equivalent exists

SimpleFOC computes the zero-sequence by min-max injection — `center -= (Umax+Umin)/2`, a few compares and
adds — where ours costs `atan2f` + two `sinf` + `sqrtf` per tick. Midpoint clamp and sector-based SVPWM
produce identical phase voltages in the linear region, so this is the cheapest available reduction in
control-loop runtime with no behavioural change. Not yet done; verify numerically across a full
revolution before switching.
