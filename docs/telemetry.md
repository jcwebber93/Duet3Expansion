# Closed-loop telemetry channels

The `M569.5` data collection path. Source: `ClosedLoop::CollectSample()` and
`ClosedLoop::DataTransmissionTaskLoop()` on the expansion board, `CL_RECORD_*` and
`ClosedLoopDataSizes[]` in `CANlib/src/Duet3Common.h`, and the channel table plus decoder in
RepRapFirmware's `ClosedLoop.cpp`.

---

## <a name="ordering-invariant"></a>The ordering invariant

**Bit order == `ClosedLoopDataSizes[]` order == the order fields are written to the wire == the order the
main board reads and names them.** Four separate hand-maintained lists must agree, and nothing enforces
it at compile time.

Getting it wrong shifts every subsequent CSV column by one field, which shows up as plausible-looking but
wrong numbers rather than an obvious failure. **Append new channels at the end and nowhere else.**

An earlier attempt to extend this stalled precisely here: a headings array was written in a different
order from the bits, and rather than reorder the array the code tried to shuffle the bitmask to
compensate. That shuffle — the "bitshift filter" — was a workaround for self-inflicted disorder, and it
is why the effort was abandoned. The main board now drives both its heading writer and its decoder from
one table, so two of the four lists became one.

## <a name="budget"></a>Sample budget

A sample must fit in **56 bytes** — `CanMessageClosedLoopData` is `4 + 4 + data[56]`, exactly the 64-byte
CAN-FD frame. All 24 channels enabled at once would be 60 bytes, so the main board rejects an
over-budget filter with an error naming the byte count. The expansion board also guards its packing loop,
since an older host could bypass that check.

---

## <a name="sign-convention"></a>Sign convention

Position and PID channels are multiplied by `recordMultiplier` — the direction multiplier for DC servo
and FOC drives, `1.0` for a classic stepper — so they are logged in **direction-adjusted logical space**.

The reason: `currentPositionError`, which is the loop's actual convergence criterion, is already in
logical space. Logging measured position in raw encoder space against a target in logical space made a
perfectly-tracking motor show Target and Measured moving in visually opposite directions whenever `S0`
was set, so a correctly-functioning drive looked like it was spinning the wrong way.

**The seven current channels (bits 17–23) deliberately take no multiplier.** They are electrical
quantities describing what the windings are doing, and that does not change with which way the axis is
declared to turn — `Ia`/`Ib`/`Ic` obviously so, and the d/q pairs because they are those same currents
resolved onto the rotor's own axes.

> An earlier draft applied the multiplier to `Iq` alone, on the grounds that its sign follows commanded
> torque. That is wrong: `Vq` derives from the same commanded torque, so with `S1` set `Iq` would print
> with the opposite sign to the `Vq` that produced it, and the pair would look like a drive fighting
> itself. Keep the whole quadruple in the physical rotor frame.

---

## <a name="phase-shift"></a>`Phase Shift` (bit 10) has never had a source

This channel writes a **literal `0`**, and always has. There is no phase-shift variable anywhere in the
firmware; the channel claims a bit, costs 2 bytes per sample and produces a column that is always zero.

It was briefly repurposed to carry `vel_measured` during FOC debugging, when nothing else exposed
velocity, and reverted once `CL_RECORD_MEASURED_VELOCITY` (bit 16) became real. Worth knowing because
logs captured in that window have a "Phase Shift" column containing velocity in raw counts/tick.

All three sides now agree it is a `uint16_t`, matching the two step-phase channels either side of it —
during the repurposing the expansion board wrote `PutF16` while the main board decoded `f16`, and the
revert to `PutU16` briefly left the main board still decoding a half-float. That was invisible only
because `0x0000` is `+0.0` in both representations.

## <a name="velocity-units"></a>`Measured Velocity` (bit 16) has drive-type-dependent units

It is the only channel written as a block rather than a one-liner, because it branches:

- DC servo and FOC: converted to **mm/s** for readable charting
- classic stepper: raw `vel_measured`, in counts/tick

One heading, two units, decided by configuration the chart cannot see.

The branch also omits `recordMultiplier` on the stepper path. That is harmless *today* — `recordMultiplier`
is `1.0` for a classic stepper by construction, so the two conditions agree — but it couples two
separately written conditions. Add a drive type to one and forget the other and the multiplier is
silently dropped.
