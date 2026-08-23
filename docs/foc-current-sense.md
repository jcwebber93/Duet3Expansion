# FOC current sensing

How `FocCurrentSense` came to be shaped the way it is. Source: `src/ClosedLoop/FocCurrentSense.{h,cpp}`,
with the Clarke/Park transforms in `FocController::MeasureDq()` and the frame calibration driven from
`ClosedLoop::SetClosedLoopEnabled()`.

Hardware throughout: SAMME51 + TI DRV8316 EVM, 24 V, Lin Engineering BL17E19-01D with an E5-1000 encoder.

---

## <a name="adc-ownership"></a>Why ADC0 is taken wholesale

CoreN2G's `AnalogIn` is a millisecond-rate round-robin configured for thermistors — 16-bit with 64×
hardware averaging, well over 100 µs per reading against a 50 µs PWM period. It cannot do
PWM-synchronous sampling, and sharing the device would mean fighting it for `INPUTCTRL` on every scan.

Taking it outright turned out to be nearly free. `AnalogIn` only touches ADC0 hardware when a channel is
first *enabled* (`AdcClass::InternalEnableChannel` → `ReInit`); the constructor does not. On this board
the thermistor path is not compiled at all (it needs `HAS_VREF_MONITOR`, which is 0) and the voltage
monitors are disabled, so the MCU temperature sensors were the only consumer. Moving them to ADC1 via
`MCU_TEMP_ADC_NUMBER` — defaulting to 0, so no other board is affected — is the entire claim. The clocks
are still set up for us by `AnalogIn::Init()`.

Verified on hardware: MCU temperature still reads correctly from ADC1.

**Registers are written directly, not through CoreN2G's `hri_adc_*` wrappers.** Two CoreN2G headers are
unusable from this project, both because they are written for a different device variant: `Timers.h`
unconditionally references TC4/TC5/TCC3/TCC4, and the entire body of `hri_adc_e54.h` sits inside
`#ifdef _SAME54_ADC_COMPONENT_`, which the SAME51G19A device header does not define - so it compiles away
to nothing. Direct register access with explicit `SYNCBUSY` waits is used instead. The `ADC_*` bitfield
macros come from the device header and are available regardless.

---

## <a name="evctrl-enable-protected"></a>`EVCTRL` is enable-protected — the silent failure

**Symptom:** the event chain looked correctly configured in every respect and produced zero conversions.

`ADC EVCTRL` is one of the SAME5x's enable-protected registers. Writing `ADC_EVCTRL_STARTEI` to an
already-running ADC is **silently discarded** — no fault, no error, the register simply does not change.
Event-triggered conversions were therefore never armed.

Two diagnostic changes made this findable, and both are worth keeping for the same reason:

- **`rawResult[]` is seeded to the nominal zero count, not 0.** A raw count of 0 is a *valid* reading —
  full negative scale — so an uninitialised array reported a confident **−5.98 A** that looked entirely
  real in a log. A separate `haveSample[]` flag distinguishes "not yet measured" from "measured zero",
  and `M122` prints it as `gotABC` / `got---`.
- **`M122` reads the event-chain registers back** (`adcEv`, `tccOvfeo`, `evUser`) rather than trusting
  the writes landed.

It was `got---` alongside `ovr0` that identified the fault: no samples *and* no overruns means no
conversions at all, not bad ones. Given a register that discards writes silently,
configured-and-broken is otherwise indistinguishable from no-current.

`WriteEvctrl()` and `WriteDseqctrl()` bracket every write with disable / `SYNCBUSY` wait / enable.

> One prediction that was wrong and is worth not repeating: `adcEv` reads **`0x02`**, not `0x01` —
> `ADC_EVCTRL_STARTEI` is bit 1.

---

## <a name="dseq-scan"></a>The DSEQ scan, and why `AUTOSTART` is off

### What this replaced

The control loop advanced the ADC mux by hand, one phase per tick, so the three phases were sampled up
to **240 µs** apart. Two problems:

1. The rotor barely moves in 240 µs even at speed, but the **torque command changes between control
   ticks**. Three phases captured at three different current amplitudes do not form a vector — the
   reconstructed angle is wrong, and no downstream calibration recovers it.
2. A latent bug: a skipped `INPUTCTRL` write (`SYNCBUSY` still set) advanced the phase index without
   advancing the mux, mislabelling every reading from then on, silently.

The evidence for (1) was that frame consistency against the commanded voltage rose monotonically as
samples were restricted to steadier current — that error being selected out:

| sample set | R (no load) | R (disturbed) |
|---|---|---|
| all moving samples | 0.196 | 0.108 |
| steadiest half | 0.325 | 0.290 |
| steadiest quarter | 0.483 | 0.396 |

### The mechanism

**The SAME5x ADC has no `SEQCTRL`** — that is a SAMD21/C21 feature, and assuming otherwise cost a build
cycle. What it has is `DSEQCTRL`, *DMA sequencing*: one DMA channel writes `INPUTCTRL` values into
`DSEQDATA`, and a second collects `RESULT`. Hence the channel pair `DmacChanFocIsenseSeq` /
`DmacChanFocIsenseRes`.

**`AUTOSTART` is deliberately left clear.** Setting it starts a conversion on every sequencer write, so
all three would run back-to-back in ~6 µs — genuinely simultaneous — but **free-running**, placing every
sample at a random point in the switching cycle. Keeping it clear means each conversion still waits for
the TCC0 overflow event, so all three are taken at the quiet point of the carrier, one per PWM period:
150 µs rather than 240 µs, with carrier synchronisation preserved.

Trading a known error for the unknown error of sampling in switching noise would not have been a good
deal. Note that SimpleFOC's inline current sense makes the opposite choice — see
[the SimpleFOC comparison](#appendix-simplefoc).

### Results do not arrive in table order

The ADC applies a sequenced `INPUTCTRL` write to a *later* conversion than the one in flight, so the
buffer is a fixed rotation of the DSEQ table — measured as exactly **240°, a two-slot rotation**, when
this replaced the hand-walked mux.

This is harmless because the frame calibration below measures the sense frame end to end and a whole-set
rotation is one of the 60° possibilities it already snaps to; the reported offset simply moved from +66°
to −174° with the same ~6° residual. **But nothing downstream may assume the buffer slots are labelled
A/B/C.**

### Measured improvement

At matched electrical speeds, current lag off the q-axis:

| elec speed (rad/s) | 240 µs spread | 150 µs spread |
|---|---|---|
| 120–250 | +35.8° | **+3.8°** |
| 250–999 | +61.1° | **+26.2°** |

This corrected an earlier misdiagnosis. The high-speed lag had been attributed to the winding's own
`atan(ωL/R)` — "the motor, not a firmware defect". **That was wrong.** A 240→150 µs change is a factor of
1.6, but the lag at 120–250 rad/s fell by a factor of nine, so most of it was the sampling defect. The
evidence for the L/R story was weaker than it was presented as: the implied time constant ranged
1.74–5.85 ms, a factor of 3.4, described at the time as "roughly constant".

---

## <a name="frame-calibration"></a>Current-sense frame calibration

### The problem

Observing the d/q frame failed its own acceptance test immediately: `|Id|` came out larger than `|Iq|`,
and the sign of `Iq` agreed with the `Vq` that produced it no more often than chance.

A mirrored sense frame is invisible to everything normally checked — phase currents look sane, sum to
zero, and calibrate normally, and the motor runs correctly because **commutation never consults the
current sense**. It only shows up once you project onto d/q.

### Why the alignment sweep is the only place this can be measured

Every other opportunity is confounded:

- During normal running the commutation angle comes from the encoder, so a wrong counts-per-electrical-rev
  or a wrong encoder direction is indistinguishable from a wrong sense mapping.
- The torque command changes between control ticks, so the three phases are captured at three different
  amplitudes.

During the sweep the commanded angle is known exactly, owes nothing to the encoder, and the torque is
constant. The only thing left that can rotate the measured current vector is the sense-to-drive mapping
itself.

### The answer

Solved from an unclipped sweep by testing all six phase permutations against both amplifier polarities:

| candidate | R | offset |
|---|---|---|
| **sense=(CBA) sign−1** | **0.999** | **−6.5°** |
| other five odd permutations | 0.999 | 60/120/180/240/300° out |
| the three even permutations | 0.009 | — |

So `drive U = −senseC`, `drive V = −senseB`, `drive W = −senseA`: an A/C channel swap plus a global
polarity inversion. The odd permutations scoring 0.999 and the even ones 0.009 is the mirror by itself —
odd permutations are reflections.

The −6.5° residual is a *lagging* angle, which is correct for an RL winding.

### Why the offset is snapped to 60°

Only multiples of 60° are physically reachable: a permutation contributes 0/120/240, amplifier polarity
another 0/180. Anything else in the measurement is the winding's load angle, and rotating that away
would be calibrating out real physics. The residual is reported and warned about above 20°, which would
mean the snap picked the wrong grid point.

### A theory withdrawn and then reinstated

The 240+180=60 explanation was proposed, then withdrawn when the sweep was found to be 99% clipped, then
reinstated when the offset reproduced at 64° saturated and 66° clean. Clipping was real and a genuine
problem, but it was never what produced the offset. Withdrawing a correct theory because a confound
turned up nearby cost a full bench cycle.

### Verified

At low speed: `|Id|/|Iq|` = 0.12, current 2.6° off the q-axis, and `Iq` agreeing in sign with `Vq` on
**128 of 128 samples**.

---

## <a name="alignment-overcurrent"></a>The alignment sweep was driving 6 A

Originally `alignTorque` was `min(0.5, W) × voltageScale` — half the bus, 12 V into a sub-ohm winding.
Both current-sense amplifiers sat on their rails (+7.22 A at ADC full scale, −5.97 A at zero, exactly as
the zero offset predicts) for **99% of the sweep**, with 177 of 205 samples reporting Kirchhoff sums of
several amps — arithmetically impossible for three real phase currents.

Under the driver's 16 A trip, so nothing ever faulted, and invisible until the sweep started sampling
current. It had been doing this on every `M569 D4` for as long as the FOC path had existed.

### The first fix did not work, and the log said so

A coarse ramp was added: step the torque up 4% at a time, measure the peak phase current after each step,
stop at 2.5 A. `M569 D4` reported `Alignment torque: 0.083 of bus (limited by current ramp)`, which looks
like it worked — 0.083 of a 24 V bus is 2 V, and 2 V into a 1.34 Ω terminal resistance is 1.5 A.

The measured phase current during the sweep that followed was **5.5–6.1 A**, still at the sense rail.

The ramp was not lying about the torque it settled on. It was measuring the wrong thing:

> **The rotor is still swinging into alignment while the ramp runs.** Applying a fixed field angle to a
> rotor that is somewhere else pulls it round, and a moving rotor generates back EMF that opposes the
> applied voltage. Every reading the ramp takes is suppressed by that motion. The current only reaches
> its steady `V/R` value once the rotor has stopped — which is *after* the ramp has already chosen a
> level and moved on.

This also explains an earlier observation that had no explanation at the time: the 120 ms pre-settle drew
4.2 A while the 400 ms sweep, at the *same commanded torque*, drew 6.4 A. Both are the same effect at
different rotor speeds.

### Alignment has a current floor as well as a ceiling

The first attempt at settle-and-correct **broke alignment entirely**, and the way it broke is the reason
every guard below exists.

It checked the settled current 40 ms into the pre-settle and again 20 ms later. The second check acted on
a reading that had not caught up with the first correction, so the two reductions compounded:

| t | peak phase current | what happened |
|---|---|---|
| 56–124 ms | 2.4 → 3.1 A | coarse ramp, settling |
| 128 ms | **1.82 A** | correction 1 |
| 148 ms | **0.84 A** | correction 2 — acting on a stale reading |

0.84 A against a 2.0 A target. The sweep that followed could not drag the rotor through a full electrical
revolution: it managed about half, then slid back, and the endpoints differed by **80 counts against an
expected 1333**.

That is not a cosmetic failure. A rejected sweep sets `TuningError::TooLittleMotion` (0x4), and only
`NeedsBasicTuning` is masked for FOC drives, so `effectiveTuningError != 0` and
`ControlMotorCurrents()` is **never called**. The drive goes completely inert with no other symptom.

So the correction is a two-sided problem. Too much current rails the sense amplifier; too little and the
alignment silently disables the drive.

### Settle, then correct — carefully

1. Coarse ramp as before — gets into the right region in ~50 ms.
2. **Pre-settle, with correction.** Hold the alignment angle. Check at 60 ms and again at 100 ms, at most
   two passes, each guarded:
   - **Deadband.** Only correct when `peak > 1.25 × target`. Aiming exactly at the target invites an
     overshoot on the low side, and low is the failure that matters.
   - **Bounded step.** One pass never reduces by more than 0.6×, so a reading taken before the rotor
     settled cannot collapse the torque.
   - **Absolute floor.** Never below 0.4 × whatever the ramp settled on, whatever the readings say.
3. Sweep at the corrected level.

Worst case across both passes is 0.36×, clamped by the floor to 0.4× — from a 3 A ramp result that is
still 1.2 A, comfortably enough to turn an unloaded rotor.

At standstill the winding is resistive, so current is nearly linear in applied voltage and one factor
lands close. Crucially the correction is *measured*, so it does not depend on knowing the relationship
between the `ApplyTorque()` argument and the resulting current — which the numbers above show is not the
naive `fraction × Vbus / R`.

This runs **inside the existing 120 ms pre-settle window**, so it costs nothing against the ~900 ms CAN
reply budget.

### The target

`min(3.0 A, focMaxCurrent × 1.5)`. Alignment is a brief static hold that has to overcome cogging from a
standstill, so it gets headroom over the running limit rather than being capped by it — a rig configured
`F...:2.0` aligns at 3 A, not 2 A.

The bracketing evidence: **5.5–6.1 A worked** (but railed the sense), **0.85 A failed**. 3 A sits between
them with margin on both sides.

Two other guards in the same area:

- **A blind ramp no longer means full torque.** If the ramp completes without ever reading over target —
  exactly the case where the measurement is broken and nothing is limiting the current — it used to leave
  `alignTorque` at the full `W` ceiling. It now falls back to a quarter of it.
- **Saturation during a correction backs off by one bounded step.** If the amplifiers are clipping there
  is no reading to scale from, only the knowledge that we are past the sense range.

`M569 D4` reports the outcome, and says what to change if the sweep is rejected:

```
Alignment torque: 0.055 of bus (target 3.00A, 1 settle correction)
Alignment: 1 electrical rev = 1294 counts measured (direction forward), 1333 counts from C/L
```

A saturated sweep still **refuses to report a frame** rather than reporting a confident wrong one. The
first version of this measurement returned "confidence 1.00" from almost entirely clipped data, which is
the worst possible failure mode — a precise, plausible, wrong answer.

---

## <a name="saturation"></a>Saturation is a measurement, not a gap

`FocCurrentSense` tracks two independent reasons a scan cannot be used, and the distinction matters more
than it first appears:

| counter | what it means | what the drive should do |
|---|---|---|
| `staleScans` | no coherent scan has arrived recently | nothing is known; hold or back off |
| `saturatedScans` | the amplifiers are clipping | the current is **past the sense range** |

The second is not an absence of information. It is the drive reporting an overcurrent in the only way it
can. Treating it as "no data" is what produced the worst failure this project has had.

Both are counted, and both count against `IsMeasurementFresh()`, but on **different thresholds**:
saturation trips at 3 scans, staleness at 9. That ordering is deliberate. Saturated scans also increment
`staleScans`, so if freshness depended on staleness alone, the loop would spend another ~480 µs feeding
on the last pre-saturation reading — a reading that is *too low*, which makes the loop command **more**
voltage into an overcurrent. The separate, shorter saturation threshold closes that window.

`IsSaturated()` exposes the state to the control loop, which uses it to decide how fast to collapse its
output — see [foc-current-control.md](foc-current-control.md#fallback).

---

## <a name="staleness"></a>Why a stale measurement must not be reused

**This one locked the test rig at 7 A**, and the cause was a policy that is correct for telemetry and
dangerous for feedback.

The Kirchhoff check discards incoherent scans and keeps the last good one. When the drive saturates an
amplifier, *every* subsequent scan is rejected — so the feedback freezes at whatever it last saw. The
integrator then winds against an error that can no longer change, pins the output, and holds the motor at
the current that caused the saturation.

In the log, `Ia`/`Ib`/`Ic` are byte-identical for thousands of consecutive samples while `ok=421
rej=1670`.

The fixes:

- `staleScans` counts polls since the last accepted scan; `IsMeasurementFresh()` goes false after 8.
- Saturation is detected **separately** and on a shorter threshold — see
  [Saturation is a measurement](#saturation) above.
- The current loops stop regulating on an unusable measurement **and clear their integrators**. Holding
  them would re-enter the fault the instant measurement recovered, since the held value is exactly what
  caused it.
- What they fall back to is bounded and decaying rather than full voltage authority — see
  [foc-current-control.md](foc-current-control.md#fallback). Without that last piece the other three do
  not help: the drive leaves the loops, applies full voltage, stays over the sense range, and never gets
  a usable measurement again.
---

## <a name="appendix-simplefoc"></a>Appendix: how SimpleFOC does it

Comparison against the vendored `Arduino-FOC` (`common/base_classes/CurrentSense.cpp`,
`current_sense/InlineCurrentSense.cpp`).

**Transforms are identical.** SimpleFOC's Clarke removes the mean explicitly (`mid = (a+b+c)/3`) where
ours uses `(2a−b−c)/3` — the same expression, and both reject common-mode amplifier offset, which is why
we use the three-phase form rather than the two-phase shortcut. Park matches exactly.

**Sampling is the real difference.** `InlineCurrentSense::getPhaseCurrents()` is three blocking
`analogRead`s at whatever instant `loopFOC()` runs — **no PWM synchronisation at all** for inline sensing
(only low-side gets `_driverSyncLowSide`). So SimpleFOC's three reads are microseconds apart but at an
arbitrary point in the switching cycle; ours are at the quiet point but spread over 150 µs. That is
precisely why SimpleFOC enables `LPF_current_d/q` by default — it is filtering ripple we largely avoid by
construction.

**Frame calibration differs in kind.** `CurrentSense::alignBLDCDriver()` drives each phase with DC,
identifies which channel reads highest, and swaps the pin/offset/gain assignment — resolving permutation
and polarity *per channel*. Ours measures the composite rotation/reflection from the AC sweep. Theirs
diagnoses the exact wiring fault; ours detects any non-isometric error through the confidence metric but
cannot say which channel caused it.
