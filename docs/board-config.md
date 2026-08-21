# SAMME51 board configuration

Reasoning behind the FOC-related pin and peripheral allocation in `src/Config/SAMME51.h`.

---

## <a name="two-configurations"></a>Two hardware configurations

Both use the same encoder, commutation, alignment and control-loop code. They differ only in how much
the firmware knows about the gate driver between the PWM outputs and the motor.

**`SUPPORT_DRV8316_SPI 1` — smart driver.** A TI DRV8316 on SERCOM4, configured and monitored over SPI:
PWM mode, slew rate, OCP threshold and CSA gain set by firmware, faults read back by register and decoded
in `M122`, and the driver's current-sense amplifier outputs feeding inline current sensing.
PB08/PB09/PB11 are the SPI bus, PB10 is nSCS.

**`SUPPORT_DRV8316_SPI 0` — dumb driver.** A plain 3-PWM gate driver with no digital interface, e.g. the
SimpleFOCMini (DRV8313). Firmware drives three PWM phases and reads the encoder, and that is all it can
see; the only fault visibility is the driver's open-drain nFAULT on `FocDriverFaultPin`. PB08–PB11 revert
to GPIO. No current sensing is possible.

Everything under `SUPPORT_FOC` is common to both, so the flag changes what the firmware can *observe and
configure*, never how it commutates.

Note that `M569.1 F` (current-loop gains) is still parsed and stored in the dumb-driver build — it is
under `SUPPORT_FOC`, not `SUPPORT_DRV8316_SPI` — but `FocCurrentModeActive()` returns false
unconditionally there, so it has no effect and nothing reports that fact. Remove it from configs used
with that build.

---

## <a name="adc-allocation"></a>ADC allocation

`FocCurrentSense` owns ADC0 outright. The three sense pins are **ADC0-only** —

| pin | input | signal |
|---|---|---|
| PA02 | ADC0/AIN0 | ISENA |
| PA06 | ADC0/AIN6 | ISENB |
| PA07 | ADC0/AIN7 | ISENC |

— which is also why simultaneous sampling via two ADCs in parallel is not available on this board without
rewiring. PA06 and PA07 previously declared `AdcInput::none` in the PinTable despite the silicon having
inputs there, which made them unusable; correcting that took the free ADC-capable pin count from three to
five and is what made this allocation possible.

All three on one ADC means a single TCC0 overflow event can trigger one DMA-sequenced conversion set.
Sampling all three rather than deriving the third from `Ia + Ib + Ic = 0` costs one conversion and buys
that identity as a live sanity check — which the saturation and coherence detection then depends on.

**MCU temperature moved to ADC1** (`MCU_TEMP_ADC_NUMBER`, defaulting to 0 so no other board is affected).
Enabling any channel on an ADC is what makes CoreN2G's `AnalogIn` claim and configure it, so moving the
only ADC0 consumer is the whole of what "claiming ADC0" requires. Verified on hardware: MCU temperature
still reads correctly, and tracks, from ADC1.

---

## <a name="vref"></a>ADC reference: why internal is the default

The DRV8316's CSA outputs are bidirectional, centred on its own VREF/2 and swinging over 0..VREF. The TI
EVM supplies VREF at 3.0 V, so zero current sits at 1.5 V and, at the 0.25 V/A gain set in
`DRV8316::Init()`, full scale is ±6.0 A.

Two arrangements are possible, selected by `FOC_CURRENT_SENSE_EXTERNAL_VREF`:

**0 (default) — internal VDDANA reference.** Nothing to wire. The CSA's 0..3.0 V swing sits inside the
3.3 V range, so zero current lands near 1862 counts rather than mid-scale. VREF and VDDANA are
independent regulators so their ratio can drift, but the zero-offset calibration performed at every
alignment measures the actual zero point and absorbs it.

**1 — ratiometric, VREF driven into ANAREF/VREFA on PA03.** Signal and reference move together, so zero
sits at exactly mid-scale by construction and stays there regardless of drift. Slightly better in
principle.

Ratiometric was the original intent, but **the TI EVM does not bring VREF out on a header pin** — its
VREF header pin is an *input*, for when an external MCU supplies the reference. Using it needs a wire
from the EVM's VREF test point to PA03. Leaving PA03 floating while the firmware expected a reference
there produced A=B=C=4095 on all three channels, which is worth recognising: a railed reading on *every*
channel simultaneously points at the reference, not the signals.

PA03 is reserved in the PinTable either way, since it cannot be a general-purpose port while it is a
candidate reference input.

---

## <a name="dma-channels"></a>DMA channels

Channels 6 and 7 are the phase-current scan pair (`DmacChanFocIsenseSeq` writes `INPUTCTRL` into
`DSEQDATA`, `DmacChanFocIsenseRes` collects `RESULT`). Deliberately **not** `DmacChanAdc0Rx` (channel 0),
which belongs to CoreN2G's `AnalogIn`: nothing enables an ADC0 channel on this board any more, but
sharing it would make the current sense depend on that staying true.

---

## <a name="pwm-carrier"></a>PWM carrier

`FOC_CENTRE_ALIGNED_PWM` puts all three phases on TCC0 in dual-slope mode with the overflow at BOTTOM,
which is the centre of the window where all three low-side FETs conduct — the only point in the period
where phase current can be sampled without switching noise.

`FOC_PWM_EVENT_DEBUG_PIN` (off by default) pulses a GPIO from the same overflow event, for confirming on
a scope that the sample instant lands where it should. Verified that way: the marker sits at the centre
of the low portion of all three phase outputs, carrier at 20 kHz.

Note the phase-to-TCC-channel mapping is *reversed* (U→WO[2], V→WO[1], W→WO[0]).
`FocController::WritePhaseDuties()` routes through the named channel constants, so this is correct — but
it looks like a bug at a glance and was investigated as one while chasing the mirrored current-sense
frame.
