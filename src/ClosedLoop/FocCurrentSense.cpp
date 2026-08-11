/*
 * FocCurrentSense.cpp
 */

#include "FocCurrentSense.h"

#if SUPPORT_FOC && SUPPORT_DRV8316_SPI

#include "DRV8316.h"
#include <Hardware/IoPorts.h>
#include <Movement/StepTimer.h>
#include <DmacManager.h>

// Registers are written directly rather than through CoreN2G's hri_adc_* wrappers. Those live in
// hri_adc_e54.h, whose entire body is inside #ifdef _SAME54_ADC_COMPONENT_ - the SAME51G19A device
// header defines a different component macro, so the header compiles away to nothing here. The ADC_*
// bitfield macros come from the device header and are available regardless.

namespace FocCurrentSense
{
	namespace
	{
		bool initialised = false;

		// Doubles as the interlock that keeps Poll() (control loop task) off the ADC while
		// CalibrateZeroOffset() and AppendDiagnostics() (command task) drive it in software.
		volatile bool synchronised = false;
		uint16_t zeroOffset[NumPhases] = { 0, 0, 0 };
		bool zeroOffsetValid = false;

		unsigned int currentPhase = 0;			// which phase the ADC mux is currently pointed at
		uint32_t overruns = 0;					// conversions discarded because the loop did not collect them in time
		volatile uint32_t collectedCount = 0;	// total conversions collected; RefreshAllPhases() waits on this advancing

		// One scan of the three phases lands here.
		//
		// NOT necessarily in the order the DSEQ table asks for them. The ADC applies a sequenced INPUTCTRL
		// write to a later conversion than the one in flight when it arrives, so the buffer is a rotation
		// of the table by some fixed amount. Measured on hardware: moving from the hand-walked mux to this
		// scan shifted the sense-frame offset by exactly 240 degrees, which is a two-slot rotation.
		//
		// This is harmless because the alignment sweep measures the sense frame end to end - through the
		// ADC, the DMA and the Clarke transform - and a whole-set rotation is one of the 60-degree
		// possibilities it already snaps to. It is emphatically NOT safe to hand-label these as A/B/C and
		// assume the labels are right; nothing downstream should do so.
		alignas(4) volatile uint16_t dmaBuffer[NumPhases] = { 0, 0, 0 };
		alignas(4) uint32_t dseqTable[NumPhases] = { 0, 0, 0 };		// INPUTCTRL values, filled in by Init()
		uint32_t acceptedSets = 0, rejectedSets = 0, saturatedSets = 0;

		// Polls since the last accepted scan. A control loop must never keep feeding on the last good
		// reading: when the drive saturates an amplifier, EVERY subsequent scan is rejected, so a policy of
		// "keep the previous set" freezes the feedback at whatever it last saw. An integrator then winds
		// against an error that can no longer change, pins the output, and holds the motor at the current
		// that caused the saturation. That is not hypothetical - it locked the test rig at 7A.
		volatile unsigned int staleScans = 0;
		constexpr unsigned int MaxStaleScans = 8;			// ~640us at the control tick

		// A reading this close to either rail is not a measurement, it is a clamp, and on these amplifiers
		// it means the current is past the sense range entirely.
		constexpr uint16_t AdcRailMargin = 16;

		inline void NoteStaleScan() noexcept
		{
			if (staleScans < MaxStaleScans + 1) { staleScans = staleScans + 1; }
		}

		// Kirchhoff tolerance for accepting a set, in raw counts. At ~3.2mA per count this is about 0.5A,
		// far above the ~10-15 counts a healthy set shows (amplifier offset and gain mismatch) and far
		// below the hundreds a mis-aligned or saturated set produces.
		constexpr uint16_t MaxKirchhoffCounts = 150;

		// (Re)arm both halves of the scan for one three-conversion block: the sequencer channel that feeds
		// INPUTCTRL, and the result channel that collects RESULT.
		//
		// Order matters. The result channel is armed first so that it is already waiting when the first
		// conversion completes; arming the sequencer first would leave a window in which a result could be
		// produced with nowhere to go, putting the buffer one beat out of step with the table for the rest
		// of the block. That misalignment is exactly what the Kirchhoff test in Poll() exists to catch, but
		// it is better not to create it.
		void ArmScanDma() noexcept
		{
			DmacManager::DisableChannel(DmacChanFocIsenseRes);
			DmacManager::DisableChannel(DmacChanFocIsenseSeq);

			DmacManager::SetSourceAddress(DmacChanFocIsenseRes, &ADC0->RESULT.reg);
			DmacManager::SetDestinationAddress(DmacChanFocIsenseRes, dmaBuffer);
			DmacManager::SetDataLength(DmacChanFocIsenseRes, NumPhases);	// must come after the addresses
			DmacManager::EnableChannel(DmacChanFocIsenseRes, DmacPrioAdcRx);

			DmacManager::SetSourceAddress(DmacChanFocIsenseSeq, dseqTable);
			DmacManager::SetDestinationAddress(DmacChanFocIsenseSeq, &ADC0->DSEQDATA.reg);
			DmacManager::SetDataLength(DmacChanFocIsenseSeq, NumPhases);
			DmacManager::EnableChannel(DmacChanFocIsenseSeq, DmacPrioAdcRx);
		}

		constexpr Pin SensePins[NumPhases] = { FocCurrentSenseAPin, FocCurrentSenseBPin, FocCurrentSenseCPin };
		uint8_t senseChannel[NumPhases] = { 0, 0, 0 };		// ADC0 input numbers, resolved in Init()

		// 12-bit conversions, no hardware averaging. The AnalogIn service uses 16-bit with 64x averaging
		// for thermistors; that is over 100us per reading, against a 50us PWM period.
		constexpr uint32_t AdcResolutionBits = 12;
		constexpr uint32_t AdcFullScale = (1u << AdcResolutionBits);		// 4096

		// The CSA outputs are driven op-amp outputs, so a short sample time is sufficient. Prescaler is
		// held at DIV8 (60MHz GCLK -> 7.5MHz ADC clock, max allowed 16MHz); the SAME5x errata also
		// requires prescaler <= 8 for the DMA sequencing that the synchronised stage will use.
		constexpr uint32_t SampleLength = 4;

		// Volts at ADC full scale, and hence the expected zero-current reading.
		//
		// Ratiometric (external VREF): the DRV8316's VREF drives both the CSA mid-point and the MCU's
		// reference, so zero current lands at exactly mid-scale by construction and stays there as VREF
		// drifts. Internal reference: the ADC measures against VDDANA instead, so zero lands wherever
		// VREF/2 falls within that range - near 1862 counts for 1.5 V against 3.3 V - and the ratio of
		// the two supplies can drift. The zero-offset calibration measures the real value either way;
		// ExpectedZero only exists to sanity-check it.
#if FOC_CURRENT_SENSE_EXTERNAL_VREF
		constexpr float AdcReferenceVolts = FocCurrentSenseVref;
#else
		constexpr float AdcReferenceVolts = 3.3;					// VDDANA
#endif
		constexpr uint16_t ExpectedZero = (uint16_t)(((FocCurrentSenseVref / 2) / AdcReferenceVolts) * AdcFullScale);
		constexpr uint16_t MaxZeroDeviation = AdcFullScale / 8;		// +-12.5% of full scale

		constexpr unsigned int CalibrationSamples = 256;

		// Latest raw conversion per phase. Seeded to the nominal zero rather than 0: a raw count of 0 is
		// not "no reading", it is a legitimate value meaning full negative scale, so leaving these at 0
		// before any conversion has happened reports about -6A of phantom current that looks entirely
		// real in a log. haveSample distinguishes the two cases for diagnostics.
		volatile uint16_t rawResult[NumPhases] = { ExpectedZero, ExpectedZero, ExpectedZero };
		volatile bool haveSample[NumPhases] = { false, false, false };

		inline void WaitForSync(uint32_t bits) noexcept
		{
			while ((ADC0->SYNCBUSY.reg & bits) != 0) { }
		}

		// Write EVCTRL with the ADC disabled around the write.
		//
		// EVCTRL is one of the SAME5x ADC's enable-protected registers: writing it while CTRLA.ENABLE is
		// set is silently discarded - no fault, no indication, the register simply does not change. That
		// is what stopped event-triggered conversions from ever starting: STARTEI was written to an
		// already-running ADC and never took, so no conversion was ever triggered, which showed up as no
		// samples collected AND no overruns.
		void WriteEvctrl(uint8_t val) noexcept
		{
			const bool wasEnabled = (ADC0->CTRLA.reg & ADC_CTRLA_ENABLE) != 0;
			if (wasEnabled)
			{
				ADC0->CTRLA.reg &= (uint16_t)~ADC_CTRLA_ENABLE;
				WaitForSync(ADC_SYNCBUSY_ENABLE);
			}
			ADC0->EVCTRL.reg = val;
			if (wasEnabled)
			{
				ADC0->CTRLA.reg |= ADC_CTRLA_ENABLE;
				WaitForSync(ADC_SYNCBUSY_ENABLE);
			}
		}

		// DSEQCTRL selects which ADC registers the sequencer DMA updates before each conversion. Bracketed
		// with disable/enable for the same reason as EVCTRL above - the enable-protected list is easy to
		// misread, and a silently discarded write here would look exactly like "the scan is not running".
		void WriteDseqctrl(uint32_t val) noexcept
		{
			const bool wasEnabled = (ADC0->CTRLA.reg & ADC_CTRLA_ENABLE) != 0;
			if (wasEnabled)
			{
				ADC0->CTRLA.reg &= (uint16_t)~ADC_CTRLA_ENABLE;
				WaitForSync(ADC_SYNCBUSY_ENABLE);
			}
			ADC0->DSEQCTRL.reg = val;
			if (wasEnabled)
			{
				ADC0->CTRLA.reg |= ADC_CTRLA_ENABLE;
				WaitForSync(ADC_SYNCBUSY_ENABLE);
			}
		}

		// Zero-current reference for a phase, in raw counts: the calibrated value when we have one, the
		// nominal otherwise.
		inline uint16_t PhaseZero(unsigned int phase) noexcept
		{
			return (zeroOffsetValid) ? zeroOffset[phase] : ExpectedZero;
		}
	}
}

bool FocCurrentSense::Init() noexcept
{
	// Resolve the pin numbers to ADC input channels, and put the pins into analog mode. PA03 carries the
	// external reference rather than a signal, so it is configured as analog but never converted.
	for (unsigned int i = 0; i < NumPhases; ++i)
	{
		IoPort::SetPinMode(SensePins[i], PinMode::AIN);
		senseChannel[i] = (uint8_t)GetInputNumber(PinToAdcChannel(SensePins[i]));
	}
#if FOC_CURRENT_SENSE_EXTERNAL_VREF
	IoPort::SetPinMode(FocCurrentSenseVrefPin, PinMode::AIN);
#endif

	// MCLK and the GCLK channel are already enabled for both ADCs by AnalogIn::Init(), so only the
	// peripheral itself needs configuring here.
	ADC0->CTRLA.reg = ADC_CTRLA_SWRST;
	WaitForSync(ADC_SYNCBUSY_SWRST);

	ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV8;				// 60MHz GCLK / 8 = 7.5MHz, max allowed is 16MHz
	ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT;
	WaitForSync(ADC_SYNCBUSY_CTRLB);
#if FOC_CURRENT_SENSE_EXTERNAL_VREF
	ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_AREFA;			// ratiometric against the DRV8316's own VREF
#else
	ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;			// VDDANA; nothing to wire
#endif
	WaitForSync(ADC_SYNCBUSY_REFCTRL);
	ADC0->EVCTRL.reg = 0;									// the synchronised stage will enable STARTEI here
	ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;
	WaitForSync(ADC_SYNCBUSY_INPUTCTRL);
	ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;			// no hardware averaging
	WaitForSync(ADC_SYNCBUSY_AVGCTRL);
	ADC0->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(SampleLength);
	WaitForSync(ADC_SYNCBUSY_SAMPCTRL);
	ADC0->WINLT.reg = 0;
	WaitForSync(ADC_SYNCBUSY_WINLT);
	ADC0->WINUT.reg = 0xFFFF;
	WaitForSync(ADC_SYNCBUSY_WINUT);
	ADC0->GAINCORR.reg = 1u << 11;
	WaitForSync(ADC_SYNCBUSY_GAINCORR);
	ADC0->OFFSETCORR.reg = 0;
	WaitForSync(ADC_SYNCBUSY_OFFSETCORR);
	ADC0->DSEQCTRL.reg = 0;
	ADC0->DBGCTRL.reg = 0;

	// Load the factory bias calibration, same as CoreN2G's AnalogIn does. Skipping this measurably
	// degrades linearity.
	{
		const uint32_t biasComp = (*reinterpret_cast<const uint32_t*>(ADC0_FUSES_BIASCOMP_ADDR) & ADC0_FUSES_BIASCOMP_Msk) >> ADC0_FUSES_BIASCOMP_Pos;
		const uint32_t biasRefbuf = (*reinterpret_cast<const uint32_t*>(ADC0_FUSES_BIASREFBUF_ADDR) & ADC0_FUSES_BIASREFBUF_Msk) >> ADC0_FUSES_BIASREFBUF_Pos;
		const uint32_t biasR2R = (*reinterpret_cast<const uint32_t*>(ADC0_FUSES_BIASR2R_ADDR) & ADC0_FUSES_BIASR2R_Msk) >> ADC0_FUSES_BIASR2R_Pos;
		ADC0->CALIB.reg = (uint16_t)(ADC_CALIB_BIASCOMP(biasComp) | ADC_CALIB_BIASREFBUF(biasRefbuf) | ADC_CALIB_BIASR2R(biasR2R));
	}

	ADC0->INTENCLR.reg = 0x07;								// no interrupts: CoreN2G owns the ADC0_1 (RESRDY) vector
	ADC0->CTRLA.reg |= ADC_CTRLA_ENABLE;
	WaitForSync(ADC_SYNCBUSY_ENABLE);

	// Prove the ADC responds before claiming success. A conversion that never completes means the
	// peripheral is not clocked or not enabled; one that returns full scale on every channel usually
	// means the reference is missing.
	initialised = true;
	const uint16_t probe = ReadRaw(0);
	if (probe == 0xFFFF)
	{
		initialised = false;
	}
	return initialised;
}

bool FocCurrentSense::IsInitialised() noexcept
{
	return initialised;
}

uint16_t FocCurrentSense::ReadRaw(unsigned int phase) noexcept
{
	if (!initialised || phase >= NumPhases)
	{
		return 0;
	}

	ADC0->INPUTCTRL.reg = (uint16_t)(ADC_INPUTCTRL_MUXNEG_GND | (uint16_t)senseChannel[phase]);
	WaitForSync(ADC_SYNCBUSY_INPUTCTRL);

	// The first conversion after changing the mux is taken on a partially-settled input, so discard it.
	// This costs ~3us and only matters on this polled path; the synchronised stage keeps each channel
	// selected across whole PWM periods.
	for (unsigned int i = 0; i < 2; ++i)
	{
		ADC0->INTFLAG.reg = ADC_INTFLAG_RESRDY;
		WaitForSync(ADC_SYNCBUSY_SWTRIG);
		ADC0->SWTRIG.reg = ADC_SWTRIG_START;
		uint32_t timeout = 100000;
		while ((ADC0->INTFLAG.reg & ADC_INTFLAG_RESRDY) == 0)
		{
			if (--timeout == 0)
			{
				return 0xFFFF;						// conversion never completed
			}
		}
	}
	return (uint16_t)ADC0->RESULT.reg;
}

void FocCurrentSense::StartSynchronisedSampling() noexcept
{
	if (!initialised)
	{
		return;
	}

	// Route TCC0 overflow to the ADC's start-conversion input. TCC0 is configured for dual-slope PWM
	// with the overflow at BOTTOM (see FocController::InitCentreAlignedPwm), which is the centre of the
	// window where all three low-side FETs conduct - the only point in the period where phase current can
	// be measured without switching noise. Verified on a scope by pulsing a GPIO from this same event.
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;
	GCLK->PCHCTRL[EVSYS_GCLK_ID_0 + FocCurrentSenseEventChannel].reg = GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN;
	EVSYS->Channel[FocCurrentSenseEventChannel].CHANNEL.reg =
		  EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_TCC0_OVF)
		| EVSYS_CHANNEL_PATH_RESYNCHRONIZED
		| EVSYS_CHANNEL_EDGSEL_RISING_EDGE;
	EVSYS->Channel[FocCurrentSenseEventChannel].CHINTENCLR.reg = EVSYS_CHINTENCLR_EVD | EVSYS_CHINTENCLR_OVR;
	EVSYS->USER[EVSYS_ID_USER_ADC0_START].reg = FocCurrentSenseEventChannel + 1;	// 0 means "not connected"

	// Hand the mux over to DMA sequencing.
	//
	// What this replaces: the control loop advanced the mux by hand, one phase per tick, so the three
	// phases were sampled up to 240us apart. The rotor barely moves in that time even at speed, but the
	// TORQUE COMMAND changes between control ticks, and three phases captured at three different current
	// amplitudes do not form a vector - the reconstructed angle is wrong, and no downstream calibration
	// recovers it. Measured on hardware: frame consistency against the commanded voltage rose monotonically
	// as samples were restricted to steadier current, which is that error being selected out. It also had a
	// latent bug, in that a skipped INPUTCTRL write (SYNCBUSY still set) advanced the phase index without
	// advancing the mux, mislabelling every reading from then on.
	//
	// The SAME5x ADC cannot sequence its own mux - it has no SEQCTRL, unlike the SAMD21/C21. What it has is
	// DMA sequencing: one DMA channel writes INPUTCTRL values into DSEQDATA, the ADC applies each one to a
	// subsequent conversion, and a second channel collects RESULT. The scan is therefore hardware-driven,
	// which removes the mux race, and the phase-to-slot relationship is fixed rather than drifting - though
	// it is a rotation of the table, not the table itself; see the note on dmaBuffer.
	//
	// AUTOSTART is deliberately LEFT OFF. With it set, each DSEQ write immediately starts a conversion, so
	// the three would run back-to-back in ~6us - genuinely simultaneous - but free-running, unsynchronised
	// to the PWM carrier, which puts every sample somewhere random in the switching cycle. Keeping
	// AUTOSTART clear means each conversion still waits for the TCC0 overflow event, so all three are taken
	// at the quiet point of the carrier, one per PWM period: a 150us spread rather than 240us.
	//
	// That is an improvement, not a cure. Getting carrier-synchronised AND simultaneous needs either the
	// two ADCs converting in parallel (which this board cannot do - PA02/PA06/PA07 are all ADC0-only) or
	// arming the scan from the carrier event itself via the DMAC's event input. Until then the current
	// loop's bandwidth has to respect the residual spread.
	for (unsigned int phase = 0; phase < NumPhases; ++phase)
	{
		dseqTable[phase] = (uint32_t)(ADC_INPUTCTRL_MUXNEG_GND | (uint16_t)senseChannel[phase]);
	}

	DmacManager::SetBtctrl(DmacChanFocIsenseRes,
			DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_DSTINC
			| DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_BLOCKACT_NOACT);
	DmacManager::SetTriggerSource(DmacChanFocIsenseRes, DmaTrigSource::adc0_resrdy);

	DmacManager::SetBtctrl(DmacChanFocIsenseSeq,
			DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC
			| DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_NOACT);
	DmacManager::SetTriggerSource(DmacChanFocIsenseSeq, DmaTrigSource::adc0_seq);

	currentPhase = 0;
	ADC0->INTFLAG.reg = ADC_INTFLAG_RESRDY | ADC_INTFLAG_OVERRUN;
	WriteDseqctrl(ADC_DSEQCTRL_INPUTCTRL);				// INPUTCTRL only, AUTOSTART clear - see above
	WriteEvctrl(ADC_EVCTRL_STARTEI);
	ArmScanDma();
	synchronised = true;
}


bool FocCurrentSense::IsSynchronised() noexcept
{
	return synchronised;
}

void FocCurrentSense::Poll() noexcept
{
	if (!synchronised)
	{
		return;
	}

	// Sequences fire every PWM period (~50us) while this runs on the control tick (~80us), so roughly
	// every other one is discarded. Harmless - each is a complete, self-consistent set and we simply take
	// the most recent - but counted, because a rising rate means the loop is running late.
	if ((ADC0->INTFLAG.reg & ADC_INTFLAG_OVERRUN) != 0)
	{
		ADC0->INTFLAG.reg = ADC_INTFLAG_OVERRUN;
		++overruns;
	}

	// Transfer-complete says all three beats landed. Deliberately the interrupt flag rather than
	// GetBytesTransferred(): that compares the descriptor's beat count against the write-back copy, and
	// before the channel's first trigger the write-back still holds the previous block's residue of zero,
	// which reads as "complete" for a buffer nothing has written yet.
	if ((DmacManager::GetAndClearChannelStatus(DmacChanFocIsenseRes) & (uint8_t)DmaCallbackReason::complete) == 0)
	{
		NoteStaleScan();								// still in flight; counts towards staleness either way
		return;
	}

	const uint16_t a = dmaBuffer[0];
	const uint16_t b = dmaBuffer[1];
	const uint16_t c = dmaBuffer[2];

	// Saturation first, and separately from the Kirchhoff test below, because the two mean different
	// things: a clipped reading says the current is beyond the sense range, a failed sum says the three
	// readings are not one coherent scan. Only the first is an overcurrent, and only the first should be
	// reported as one.
	constexpr uint16_t RailHigh = (uint16_t)(AdcFullScale - 1 - AdcRailMargin);
	if (a <= AdcRailMargin || a >= RailHigh || b <= AdcRailMargin || b >= RailHigh || c <= AdcRailMargin || c >= RailHigh)
	{
		++saturatedSets;
		NoteStaleScan();
		ArmScanDma();
		return;
	}

	// Three real phase currents sum to zero. A set that fails that badly is not three phase currents -
	// either the DMA armed mid-sequence and these are beats from two different ones, or an amplifier is
	// saturated. Either way the vector it would produce is meaningless, so drop it and keep the last good
	// set. Cheap in raw counts: a few subtractions, no conversion to amps.
	const int32_t sum = ((int32_t)a - (int32_t)PhaseZero(0))
					  + ((int32_t)b - (int32_t)PhaseZero(1))
					  + ((int32_t)c - (int32_t)PhaseZero(2));
	if (sum > (int32_t)MaxKirchhoffCounts || sum < -(int32_t)MaxKirchhoffCounts)
	{
		++rejectedSets;
		NoteStaleScan();
	}
	else
	{
		rawResult[0] = a; rawResult[1] = b; rawResult[2] = c;
		haveSample[0] = true; haveSample[1] = true; haveSample[2] = true;
		collectedCount = collectedCount + NumPhases;		// not ++: deprecated on a volatile in C++20
		++acceptedSets;
		staleScans = 0;
	}

	// Re-arm for the next scan. Safe here: the block just completed, so the ADC has finished its third
	// conversion and the next one cannot start until the next carrier event, up to a PWM period away.
	ArmScanDma();
}

bool FocCurrentSense::RefreshAllPhases(uint32_t timeoutUs) noexcept
{
	if (!synchronised)
	{
		return false;
	}

	// StepTimer rather than SysTick: SysTick counts down and wraps every millisecond or so, and three
	// conversions at one per PWM period already take ~150us, so a wrap inside the wait is likely.
	const uint32_t startCount = collectedCount;
	const StepTimer::Ticks deadline = (StepTimer::Ticks)(((uint64_t)timeoutUs * StepTimer::StepClockRate) / 1000000u);
	const StepTimer::Ticks startTicks = StepTimer::GetTimerTicks();
	while (collectedCount - startCount < NumPhases)
	{
		Poll();
		if ((StepTimer::Ticks)(StepTimer::GetTimerTicks() - startTicks) > deadline)
		{
			return false;
		}
	}
	return true;
}

void FocCurrentSense::GetPhaseCurrents(float& ia, float& ib, float& ic) noexcept
{
	ia = RawToAmps(0, rawResult[0]);
	ib = RawToAmps(1, rawResult[1]);
	ic = RawToAmps(2, rawResult[2]);
}

float FocCurrentSense::RawToAmps(unsigned int phase, uint16_t raw) noexcept
{
	// amps = (Vsense - Vzero) / gain. Volts are recovered against whichever reference the ADC is actually
	// measuring against, which is not necessarily the DRV8316's VREF - see AdcReferenceVolts. Each phase
	// has its own zero offset; they differ by a few counts of CSA offset error. Falls back to the nominal
	// zero before calibration has run.
	const float voltsPerCount = AdcReferenceVolts / (float)AdcFullScale;
	const float zero = (zeroOffsetValid && phase < NumPhases) ? (float)zeroOffset[phase] : (float)ExpectedZero;
	return (((float)raw - zero) * voltsPerCount) / DRV8316::CsaGainVoltsPerAmp;
}

uint16_t FocCurrentSense::GetZeroOffset(unsigned int phase) noexcept
{
	return (phase < NumPhases) ? zeroOffset[phase] : 0;
}

bool FocCurrentSense::IsCalibrated() noexcept
{
	return zeroOffsetValid;
}

bool FocCurrentSense::IsMeasurementFresh() noexcept
{
	return synchronised && staleScans <= MaxStaleScans;
}

bool FocCurrentSense::CalibrateZeroOffset(const StringRef& reply) noexcept
{
	if (!initialised)
	{
		reply.lcat("Current sense: ADC not initialised, cannot calibrate");
		return false;
	}

	// Take the ADC back off the event trigger for the duration: calibration drives conversions in
	// software, and leaving event starts enabled would interleave conversions of the wrong phase. The
	// sequencer has to go too - while DSEQCTRL selects INPUTCTRL, the sequencer DMA overwrites it before
	// every conversion, so ReadRaw() below would get whichever input the scan had reached rather than the
	// one it asked for.
	const bool wasSynchronised = synchronised;
	synchronised = false;
	WriteEvctrl(0);
	WriteDseqctrl(0);
	DmacManager::DisableChannel(DmacChanFocIsenseSeq);
	DmacManager::DisableChannel(DmacChanFocIsenseRes);

	zeroOffsetValid = false;
	bool ok = true;
	for (unsigned int phase = 0; phase < NumPhases; ++phase)
	{
		uint32_t total = 0;
		for (unsigned int i = 0; i < CalibrationSamples; ++i)
		{
			total += ReadRaw(phase);
		}
		zeroOffset[phase] = (uint16_t)(total / CalibrationSamples);

		const int32_t deviation = (int32_t)zeroOffset[phase] - (int32_t)ExpectedZero;
		if ((uint32_t)labs(deviation) > MaxZeroDeviation)
		{
			ok = false;
		}
	}

	reply.lcatf("Current sense zero: A=%u B=%u C=%u counts (expected ~%u)",
					zeroOffset[0], zeroOffset[1], zeroOffset[2], ExpectedZero);
	if (!ok)
	{
		reply.cat(" - WARNING: one or more phases are far from mid-scale; check VREF on the reference pin and the ISEN wiring");
	}
	zeroOffsetValid = ok;
	if (wasSynchronised)
	{
		StartSynchronisedSampling();
	}
	return ok;
}

void FocCurrentSense::AppendDiagnostics(const StringRef& reply) noexcept
{
	if (!initialised)
	{
		reply.lcat("Current sense: not initialised");
		return;
	}

	// In synchronised mode report the values the control loop is actually seeing. Taking fresh polled
	// conversions here would sample at an arbitrary point in the PWM period and read switching noise,
	// which is exactly what the event trigger exists to avoid.
	const bool wasSynchronised = synchronised;
	uint16_t a, b, c;
	if (wasSynchronised)
	{
		a = rawResult[0]; b = rawResult[1]; c = rawResult[2];
	}
	else
	{
		a = ReadRaw(0); b = ReadRaw(1); c = ReadRaw(2);
	}

	// Terse on purpose - see the note in ClosedLoop::InstanceDiagnostics about the shared 500-char reply.
	uint16_t polledA = 0;
	if (wasSynchronised)
	{
		// Only meaningful while synchronised: one polled conversion of phase A at an arbitrary point in
		// the PWM period, shown alongside the event-triggered value. A large disagreement means the event
		// is firing at the wrong instant, or the CSA output is not valid at that instant - as distinct
		// from a wiring or reference fault, which would break both readings equally.
		//
		// This runs on the command-processing task while Poll() runs on the control loop task, and the two
		// would otherwise be driving the same ADC concurrently - this code disables the device outright
		// inside WriteEvctrl(). Clear 'synchronised' first so Poll() returns immediately for the duration,
		// the same interlock CalibrateZeroOffset() uses. Worth doing even though currents do not yet feed
		// the control path: they will once the d/q loops close.
		synchronised = false;
		WriteEvctrl(0);
		WriteDseqctrl(0);							// ReadRaw needs INPUTCTRL honoured; the sequencer overwrites it
		// The result channel has to go too, not just the sequencer. It is armed on adc0_resrdy, so it
		// consumes RESULT the instant a conversion finishes - including ReadRaw's own software conversion,
		// whose RESRDY it clears before ReadRaw can see it. Leaving it enabled made every polled reading
		// time out and report 0xFFFF.
		DmacManager::DisableChannel(DmacChanFocIsenseSeq);
		DmacManager::DisableChannel(DmacChanFocIsenseRes);
		polledA = ReadRaw(0);
		StartSynchronisedSampling();					// restores DSEQCTRL, both DMA channels and STARTEI together
	}

	reply.lcatf("Isense raw A=%u B=%u C=%u zero~%u, %s, got%c%c%c ovr%" PRIu32,
					a, b, c, ExpectedZero,
					(wasSynchronised) ? "SYNC" : "polled", (haveSample[0]) ? 'A' : '-',
					(haveSample[1]) ? 'B' : '-', (haveSample[2]) ? 'C' : '-', overruns);
	if (wasSynchronised)
	{
		// Read the registers back rather than trusting the writes. ADC EVCTRL is enable-protected, so a
		// write to a running ADC is silently discarded; TCC EVCTRL is likewise protected once the timer
		// is enabled. adcEv should read 0x02 (STARTEI is bit 1) and tccOvfeo should be 1 - if either is 0
		// the event chain is broken at that end, which is otherwise indistinguishable from "no current".
		reply.catf(", Apoll=%u, adcEv=0x%02x tccOvfeo=%u evUser=%u",
						polledA, ADC0->EVCTRL.reg, (unsigned)TCC0->EVCTRL.bit.OVFEO,
						(unsigned)EVSYS->USER[EVSYS_ID_USER_ADC0_START].reg);
		// dseq is the DSEQCTRL read-back (expect 0x00000001: INPUTCTRL selected, AUTOSTART clear - another
		// enable-protected register, so worth confirming the write took). ok/rej are complete three-phase
		// scans accepted and rejected by the Kirchhoff test; a rejection rate that is not near zero means
		// scans are being armed out of step and the readings do not belong to the phases they are filed under.
		// sat counts scans with a phase against an ADC rail - an overcurrent past the sense range, not a
		// sequencing problem - and is what tells the two failure modes apart. stale is polls since the
		// last usable scan; any control loop stops feeding on these values once it exceeds its limit.
		reply.lcatf("Isense dseq=0x%08x ok=%" PRIu32 " rej=%" PRIu32 " sat=%" PRIu32 " stale=%u%s",
						(unsigned)ADC0->DSEQCTRL.reg, acceptedSets, rejectedSets, saturatedSets,
						staleScans, (IsMeasurementFresh()) ? "" : " STALE");
	}
	if (zeroOffsetValid)
	{
		const float ia = RawToAmps(0, a), ib = RawToAmps(1, b), ic = RawToAmps(2, c);
		reply.lcatf("Isense A=%.3fA B=%.3fA C=%.3fA sum=%.3fA", (double)ia, (double)ib, (double)ic, (double)(ia + ib + ic));
	}
	else
	{
		reply.lcat("Isense: zero offset not calibrated");
	}
}

#endif
