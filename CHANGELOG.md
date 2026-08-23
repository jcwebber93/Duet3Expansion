# Changelog

Fork of [Duet3D/Duet3Expansion](https://github.com/Duet3D/Duet3Expansion), tracking the `3.7-dev`
branch. This file records what the fork adds on top of upstream; upstream's own changes are not
duplicated here.

Format loosely follows [Keep a Changelog](https://keepachangelog.com/).

---

## [Unreleased]

Two substantial additions: **closed-loop motor control on expansion boards that have no TMC driver**, and
**field-oriented control of BLDC/PMSM motors**. The first is what makes the second possible on hardware
like the SAMME51, which has no smart driver at all.

### Added — closed-loop control without a TMC driver

Upstream drives the closed-loop and phase-stepping control law from inside the TMC51xx driver task:
`TMC51xx.cpp` calls `Move::PhaseStepControlLoop()` on every SPI cycle, immediately before writing the
phase currents. The control-loop cadence *is* the TMC SPI cycle. On a board with no TMC driver that task
does not exist, so the control law never runs — closed-loop mode is silently inert rather than broken in
any visible way.

**A dedicated closed-loop task and timer now provide that tick** (`Movement/Move.cpp`), compiled in when
`SUPPORT_CLOSED_LOOP && !HAS_SMART_DRIVERS`:

- Task `CLCtrl` at `TaskPriority::TmcClosedLoop`, driven by a `StepTimer` callback.
- Fixed **80 us period** (`ClosedLoopSleepClocks`), i.e. **12.5 kHz** against the 750 kHz step clock.
- **Absolute-deadline scheduling.** The next wake-up is `clLastWakeupTime += ClosedLoopSleepClocks`
  rather than "now + 80 us", so the period does not drift with execution time.
- **A late tick cannot hang the loop.** If `ScheduleCallback()` reports the deadline already passed, the
  task loops immediately instead of blocking on a notification that would never arrive.

Boards with a TMC driver are untouched — EXP1HCL still takes the original path through the driver task.

`M122` reports this loop under `Phase step loop runtime`, giving minimum and maximum execution time and
the achieved call frequency, so an overrunning control law is visible rather than inferred.

### Added — supporting work for boards without a smart driver

`HAS_SMART_DRIVERS` conditionals across the tree, so a board with no smart driver builds and runs:

| area | file |
|---|---|
| driver status polling, microstepping (`M350`), stall detection (`M915`), `M122` driver reporting | `CommandProcessing/CommandProcessor.cpp` |
| global driver-enable pin | `Platform/Tasks.cpp`, `Platform/Platform.cpp` |
| step-interval reporting (used only by TMC stall detect / CoolStep) | `Movement/DriveMovement.h` |
| driver temperature sensor | `Heating/Sensors/TmcDriverTemperatureSensor.{h,cpp}` |

Board defaults for `SUPPORT_FOC` and `SUPPORT_DRV8316_SPI` now live in `Config/BoardDef.h`. The build uses
`-Werror=undef`, so an undefined macro inside an `#if` is a hard error rather than a silent zero, and both
are tested in headers included by every board.

### Added — BLDC / PMSM field-oriented control

Three-phase FOC output stage (`ClosedLoop/FocController.{h,cpp}`) with SVPWM, centre-aligned PWM, and an
alignment routine that *measures* the encoder-to-electrical relationship rather than assuming it.

- **Commutation correctness.** Alignment targets 3*pi/2 so that encoder count 0 is genuinely electrical
  zero, and both the encoder-to-electrical direction and the counts per electrical revolution are
  measured by the alignment sweep. Assuming either produces zero-torque traps rather than a motor that
  merely runs backwards. See `docs/foc-commutation.md`.
- **Centre-aligned (dual-slope) PWM** on a dedicated TCC, emitting an overflow event at the carrier
  centre — a prerequisite for current sensing.
- **Open-loop assisted mode** (`M569 D5`) driving commutation from the commanded position, for verifying
  phase order and `C`/`L` scaling independently of encoder calibration.

### Added — DRV8316 gate driver and inline current sensing

- SPI configuration and fault decoding for the TI DRV8316 (`ClosedLoop/DRV8316.{h,cpp}`), reported in a
  new `M122` part.
- **PWM-synchronous inline current sensing** (`ClosedLoop/FocCurrentSense.{h,cpp}`): conversions triggered
  by the carrier overflow event and collected by DMA sequencing, so samples are taken at the quiet point
  of the switching cycle.
- **Automatic current-sense frame calibration.** The alignment sweep measures how the sense channels sit
  relative to the drive phases and corrects a mirrored or rotated mapping. A miswired sense frame is
  otherwise invisible — phase currents look sane and sum to zero, and the motor runs correctly, because
  commutation never consults the current sense.
- **Current-limited alignment.** The sweep now ramps torque under current feedback instead of applying a
  fixed half-bus pull, which on the test rig had been driving over 6 A on every `M569 D4`.

### Added — d/q current-mode control

Two PI loops regulating `Iq` to the torque demand and `Id` to zero (`M569.1 F`), making the cascade
position -> velocity -> current. Optional and off by default; the drive stays in voltage mode unless gains
are set and the current sense is calibrated.

Measured: current holds within about 3 degrees of the q-axis across the speed range, against 5 degrees
rising to 24 in voltage mode, and peak phase current on a fast move fell from roughly 4 A to 1 A for the
same motion.

### Added — configuration

New `M569.1` parameters:

| | meaning |
|---|---|
| `L` | pole pair count |
| `W` | maximum torque, as a fraction of available voltage |
| `N` / `O` | nominal supply voltage / voltage limit, in volts |
| `F` | FOC current loop `{proportional gain, integral gain, maximum phase current}` |

New motor types on `M569.1 T`: `bldc`, `stepperFoc`, `hybridStepperFoc`.

Seven new `M569.5` telemetry channels (bits 17-23): three phase currents, `Id`, `Iq`, `Vd`, `Vq`.

`M122` gains a `FOC limits:` line reporting current-mode dropouts, ticks spent over the trip, and the
fallback ceiling against the configured maximum, and `M569 D4` reports the alignment target current and
how many settle corrections it needed.

### Fixed

- **Commutation used a measured electrical period in preference to an exact one, and the error
  accumulated with distance.** The alignment sweep measures counts-per-electrical-revolution, and the
  drive adopted that figure whenever it was within a factor of two of the value derived from `C` and
  `L`. But `C` and `L` are exact integers - `C1000 L3` is 4000/3 = 1333.33 counts exactly - while
  the sweep is a 400 ms mechanical measurement that settles short every time (1293, 1294, 1301, 1320
  observed on one rig). Because the period is a *scale factor* on the commutation angle, a 3% error is
  11 degrees per electrical revolution, cumulative: on a 12,500-count move it reached 105 degrees, torque
  collapsed as its cosine, and the motor stalled at full current. Shorter moves hid it - the same error
  is only 52 degrees over 6,250 counts. Now the sweep supplies the *direction* always, and the *period*
  only when it disagrees with `C`/`L` by more than 10% (i.e. the configuration is wrong).
  See `docs/foc-commutation.md#counts-per-elec-rev`.
- **The commutation angle is computed from an exact ratio.** Even the correctly rounded integer 1333
  carries a 0.025% scale error that integrates to 27 degrees over 400,000 counts.
  `ComputeFocElectricalAngle()` now works from a numerator/denominator pair, reducing modulo the
  numerator (a whole number of electrical revolutions, so the reduction is lossless) before scaling.
  Worst-case error over the same 400,000 counts falls to 0.087 degrees - one LSB of the angle
  representation, and it does not accumulate.

- **Losing the current measurement handed the drive full voltage authority.** `FocMeasurementUsable()`
  false dropped straight to voltage mode, which clamps only to `W × O/N` — 9 V into a 1.34 Ω winding on
  the test rig, or 6.7 A. That held the current past the ±6 A sense range, so the measurement could never
  recover and the drive stayed there: **164 consecutive ticks at 7.1 A** in one log, and a second run was
  already latched before its move began. The fallback is now bounded by a ceiling re-seeded from the last
  regulated output and decayed (100 ms stale, 10 ms saturated), which contains the fault *and* provides
  the recovery path. Voltage mode by configuration (`F` absent) is unchanged.
  See `docs/foc-current-control.md#fallback`.
- **A saturated current reading was treated as missing data.** Clipping is the sense reporting an
  overcurrent, but it was folded into the staleness count and so meant "no scan arrived". It is now
  tracked separately, on a shorter threshold (3 scans against 9) — without which the loop spends ~480 µs
  feeding on the last pre-saturation reading, which reads *low* and makes it command more voltage into an
  overcurrent. See `docs/foc-current-sense.md#saturation`.
- **Exceeding the measured-current trip dropped out of current mode** — removing the loops that reduce
  current at the moment they were needed. It now stays in current mode and squeezes the output ceiling
  to 25% instead.
- **The alignment current ramp measured a moving rotor.** It reported settling at 0.083 of bus while the
  sweep that followed drew 5.5-6.1 A, still at the sense rail. The rotor is still swinging into alignment
  while the ramp runs, and the back EMF of that motion suppresses every reading it takes. The ramp is now
  a coarse first guess only, corrected by measuring a *stationary* rotor during the existing pre-settle
  window - no extra time against the CAN reply budget. Alignment has a current **floor** as well as a
  ceiling: too little and the sweep cannot turn the rotor, which sets `TuningError::TooLittleMotion` and
  gates the control law entirely, leaving the drive inert. The correction is therefore bounded by a
  deadband, a per-pass limit and an absolute floor, and the target is `min(3.0 A, F-maxCurrent x 1.5)` -
  headroom over the running limit, because alignment is a brief static hold that must overcome cogging.
  See `docs/foc-current-sense.md#alignment-overcurrent`.
- **The control law ran twice per tick for some drive types.** `InstanceControlLoop()` had three parallel
  branches that each also fell through into a shared tail, so DC servo and FOC executed
  `ControlMotorCurrents()` twice with an identical timestamp — double-integrating the I term and
  corrupting the D term. **DC servo gains tuned against the old behaviour need roughly `I` doubled and
  `D` halved.**
- **Closed-loop stall detection never ran for FOC drives.** It sat behind a tuning-error gate that a
  quadrature encoder can never clear, so `M569.1 E` thresholds were inert.
- **A braceless `if` spanning a preprocessor boundary** meant a plain `M569.1 P... T... C...` could delete
  and reallocate the encoder underneath a running control loop. `-Wmisleading-indentation` cannot catch
  this pattern, because the `#endif` breaks its heuristic.
- **Closed-loop data collection truncated the channel filter to 16 bits** on the main board, so any
  channel above bit 15 was silently dropped.

### Changed

- `M569.1 Q` no longer sets a FOC velocity limit (it remains torque-per-amp). The limit clamped the sum of
  the position correction *and* the feedforward terms, so it capped achievable feedrate and commanded
  reverse torque above the cap. `M203` already provides this limit.
- MCU temperature sensing moves to ADC1 on boards where the FOC current sense claims ADC0
  (`MCU_TEMP_ADC_NUMBER`, defaulting to 0 so no other board is affected).

### Experimental — present but not verified on hardware

Treat as unfinished:

- **DC servo output through a TMC driver's XDIRECT register** (`M569.1 U1`). Untested in any real
  capacity.
- **DC servo generally.** The double-execution fix changes its tuning, and that retune has not been done
  on hardware.
- **FOC stall detection.** Now reachable, but has never actually fired in testing.
- **`stepperFoc` and `hybridStepperFoc` output modes.** Compiled and reviewed, not run.
- **Fan control from driver temperature** is disabled tree-wide (`#if 0` in `Fans/LocalFan.cpp`) pending
  rework for boards with no driver temperature source.

### Verified on hardware

SAMME51 with a TI DRV8316 EVM, 24 V, Lin Engineering BL17E19-01D and an E5-1000 encoder: closed-loop FOC
motion, alignment repeatable to within 1% of the configured `C`/`L`, PWM-synchronous current sensing,
sense-frame calibration, and d/q current control.

### Companion repositories

Requires matching builds of:

- **CANlib** — new closed-loop telemetry channels and the `M569.1 F` parameter. Shared header, so both
  boards must be flashed together.
- **RepRapFirmware** — channel filter widened to 32 bits; the closed-loop CSV writer and decoder now
  driven from one table.
- **ClosedLoopTuningPlugin** — chart traces for the new current channels.

### Upstream merge status

Not yet rebased onto current `3.7-dev`. This list will need revisiting afterwards, particularly anywhere
upstream has also touched `Move.cpp` or the closed-loop module.
