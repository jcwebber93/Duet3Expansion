/*
 * FocController.cpp
 */

#include "FocController.h"
#include <RepRapFirmware.h>
#include <AnalogOut.h>

#if defined(FOC_CENTRE_ALIGNED_PWM) && FOC_CENTRE_ALIGNED_PWM
# include <hri_tcc_e54.h>

namespace
{
	// TCC device pointers, kept local rather than pulling in CoreN2G's Timers.h. That header is internal
	// to CoreN2G's own build and unconditionally references TC4/TC5/TCC3/TCC4, none of which exist on the
	// 48-pin SAME51G19A, so it does not compile from this project.
	volatile Tcc * const FocTccDevices[] = { TCC0, TCC1, TCC2 };
	static_assert(FocPwmTccNumber < ARRAY_SIZE(FocTccDevices));

	constexpr unsigned int FocTccGclkNum = GclkNum60MHz;		// same generator CoreN2G uses for TC/TCC
	constexpr uint32_t FocTccGclkFreq = 60000000;
}

# if FOC_PWM_EVENT_DEBUG_PIN
// Bring-up aid - see FOC_PWM_EVENT_DEBUG_PIN in the board config for what this measures and how to read
// it. Emits a short pulse at every TCC0 overflow, which is the instant the current-sense ADC trigger
// fires. The pulse width is just the handler's own duration; only its position in the PWM period matters.
static_assert(FocPwmTccNumber == 0, "the debug pulse uses TCC0_0_Handler, which is TCC0's overflow vector");

extern "C" void TCC0_0_Handler() noexcept
{
	fastDigitalWriteHigh(FocPwmEventDebugPin);
	TCC0->INTFLAG.reg = TCC_INTFLAG_OVF;			// clear the flag (write 1 to clear)
	fastDigitalWriteLow(FocPwmEventDebugPin);
}
# endif
#endif

// 3-phase BLDC / hybrid stepper constructor
FocController::FocController(Pin u, Pin v, Pin w,
		GpioPinFunction uFn, GpioPinFunction vFn, GpioPinFunction wFn,
		FocOutputMode mode) noexcept
	: phaseU(u), phaseV(v), phaseW(w), fnU(uFn), fnV(vFn), fnW(wFn),
	  in4Pin(NoPin), fnIn4(GpioPinFunction::A), outputMode(mode)
{
}

// 4PWM 2-phase stepper constructor: IN1=phaseU, IN2=phaseV, IN3=phaseW, IN4=in4Pin.
// ENA/ENB are typically hardwired high on L298N boards; pass NoPin if unused.
FocController::FocController(Pin in1, Pin in2, Pin in3, Pin in4,
		GpioPinFunction fn1, GpioPinFunction fn2,
		GpioPinFunction fn3, GpioPinFunction fn4,
		Pin ena, Pin enb) noexcept
	: phaseU(in1), phaseV(in2), phaseW(in3), fnU(fn1), fnV(fn2), fnW(fn3),
	  in4Pin(in4), fnIn4(fn4),
	  outputMode(FocOutputMode::TwoPhase4Pwm)
{
	(void)ena; (void)enb;	// ENA/ENB held high externally on most L298N boards
}

#if defined(FOC_CENTRE_ALIGNED_PWM) && FOC_CENTRE_ALIGNED_PWM

// Set up the shared TCC for dual-slope (centre-aligned) PWM.
//
// This deliberately bypasses AnalogOut::Write(). AnalogWriteTcc() in CoreN2G hard-codes
// TCC_WAVE_WAVEGEN_NPWM_Val (single slope) and is shared by every PWM consumer on the board, so it
// cannot be switched to dual slope without affecting unrelated outputs. Centre alignment is not a
// preference here - inline current sensing requires sampling in the middle of the window where all
// three low-side FETs are conducting, and only a centre-aligned carrier puts that window at a fixed,
// event-addressable point in the period.
void FocController::InitCentreAlignedPwm() noexcept
{
	volatile Tcc * const tcc = FocTccDevices[FocPwmTccNumber];

	EnableTccClock(FocPwmTccNumber, FocTccGclkNum);

	hri_tcc_clear_CTRLA_ENABLE_bit(tcc);
	while (tcc->SYNCBUSY.bit.ENABLE) { }
	hri_tcc_set_CTRLA_SWRST_bit(tcc);
	while (tcc->SYNCBUSY.bit.SWRST) { }

	// In dual-slope mode the counter ramps 0 -> PER -> 0, so one carrier period is 2*PER ticks and the
	// achievable duty resolution is PER steps. At 60 MHz GCLK and a 20 kHz carrier that is PER = 1500,
	// comfortably inside TCC0's 24-bit counter with no prescaling.
	pwmPeriod = FocTccGclkFreq / (2u * (uint32_t)pwmFreq);

	tcc->CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV1_Val;
	tcc->CTRLA.bit.RESOLUTION = 0;

	// DSBOTTOM: dual-slope PWM with the overflow event generated at BOTTOM. BOTTOM is the centre of the
	// all-low-side-on window, which is exactly where Stage 3 needs to trigger the current-sense ADC.
	hri_tcc_write_WAVE_WAVEGEN_bf(tcc, TCC_WAVE_WAVEGEN_DSBOTTOM_Val);

	tcc->PER.bit.PER = pwmPeriod;

	// Start all three phases at 50% - equal duty on every phase means zero differential voltage across
	// the windings, so the motor coasts.
	const uint32_t midpoint = pwmPeriod / 2u;
	tcc->CC[FocPhaseUTccChannel].bit.CC = midpoint;
	tcc->CC[FocPhaseVTccChannel].bit.CC = midpoint;
	tcc->CC[FocPhaseWTccChannel].bit.CC = midpoint;

	// Emit the overflow (BOTTOM) event for the current-sense ADC trigger. Harmless with no EVSYS
	// consumer attached; Stage 3 routes it to the ADC.
	tcc->EVCTRL.bit.OVFEO = 1;

#if FOC_PWM_EVENT_DEBUG_PIN
	// Mirror that same overflow to an interrupt so a GPIO pulse marks it on a scope. Deliberately set up
	// here rather than alongside the ADC, so the trigger instant can be confirmed before any ADC code
	// exists to be blamed for a bad reading.
	SetPinMode(FocPwmEventDebugPin, OUTPUT_LOW);
	tcc->INTFLAG.reg = TCC_INTFLAG_OVF;				// discard any flag set during setup
	tcc->INTENSET.reg = TCC_INTENSET_OVF;
	NVIC_DisableIRQ(TCC0_0_IRQn);
	NVIC_ClearPendingIRQ(TCC0_0_IRQn);
	NVIC_SetPriority(TCC0_0_IRQn, NvicPriorityFocPwmEventDebug);
	NVIC_EnableIRQ(TCC0_0_IRQn);
#endif

	hri_tcc_set_CTRLA_ENABLE_bit(tcc);
	while (tcc->SYNCBUSY.bit.ENABLE) { }
	tcc->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;			// without this there is a delay before PWM starts

	SetPinFunction(phaseU, fnU);
	SetPinFunction(phaseV, fnV);
	SetPinFunction(phaseW, fnW);
}

#endif

// Write the three phase duty cycles, each 0..1.
void FocController::WritePhaseDuties(float dutyU, float dutyV, float dutyW) noexcept
{
#if defined(FOC_CENTRE_ALIGNED_PWM) && FOC_CENTRE_ALIGNED_PWM
	volatile Tcc * const tcc = FocTccDevices[FocPwmTccNumber];
	// Buffered writes: the new compare values are latched together at the period boundary, so all three
	// phases change on the same carrier edge rather than tearing across an update.
	const float period = (float)pwmPeriod;
	tcc->CCBUF[FocPhaseUTccChannel].bit.CCBUF = (uint32_t)lrintf(dutyU * period);
	tcc->CCBUF[FocPhaseVTccChannel].bit.CCBUF = (uint32_t)lrintf(dutyV * period);
	tcc->CCBUF[FocPhaseWTccChannel].bit.CCBUF = (uint32_t)lrintf(dutyW * period);
#else
	AnalogOut::Write(phaseU, dutyU, pwmFreq);
	AnalogOut::Write(phaseV, dutyV, pwmFreq);
	AnalogOut::Write(phaseW, dutyW, pwmFreq);
#endif
}

void FocController::Init(PwmFrequency freq) noexcept
{
	pwmFreq = freq;

	if (outputMode == FocOutputMode::TwoPhase4Pwm)
	{
		// 4-pin H-bridge stepper output. The pins are not required to share a timer, so this mode always
		// uses the shared AnalogOut path; it has no current sensing and so no need for centre alignment.
		SetPinFunction(phaseU, fnU);	// IN1
		SetPinFunction(phaseV, fnV);	// IN2
		SetPinFunction(phaseW, fnW);	// IN3
		SetPinFunction(in4Pin, fnIn4);	// IN4
		// All inputs low — zero current through both coils
		AnalogOut::Write(phaseU, 0.0f, pwmFreq);
		AnalogOut::Write(phaseV, 0.0f, pwmFreq);
		AnalogOut::Write(phaseW, 0.0f, pwmFreq);
		AnalogOut::Write(in4Pin, 0.0f, pwmFreq);
	}
#if defined(FOC_CENTRE_ALIGNED_PWM) && FOC_CENTRE_ALIGNED_PWM
	else
	{
		InitCentreAlignedPwm();			// also sets the pin functions and starts at 50% on all phases
	}
#else
	else
	{
		SetPinFunction(phaseU, fnU);
		SetPinFunction(phaseV, fnV);
		SetPinFunction(phaseW, fnW);
		// 50% duty = zero average voltage across all windings
		WritePhaseDuties(0.5f, 0.5f, 0.5f);
	}
#endif
}

void FocController::Coast() noexcept
{
	if (outputMode == FocOutputMode::TwoPhase4Pwm)
	{
		AnalogOut::Write(phaseU, 0.0f, pwmFreq);
		AnalogOut::Write(phaseV, 0.0f, pwmFreq);
		AnalogOut::Write(phaseW, 0.0f, pwmFreq);
		AnalogOut::Write(in4Pin, 0.0f, pwmFreq);
	}
	else
	{
		WritePhaseDuties(0.5f, 0.5f, 0.5f);
		lastDutyU = lastDutyV = lastDutyW = 0.5f;
	}
}

/*static*/ void FocController::InversePark(float q, float sine, float cosine,
		float& alpha, float& beta) noexcept
{
	// d = 0 (voltage mode): alpha = -q*sin(θ), beta = q*cos(θ)
	alpha = -q * sine;
	beta  =  q * cosine;
}

void FocController::ApplyDqVoltage(float vd, float vq, uint16_t electricalAngle) noexcept
{
	if (outputMode != FocOutputMode::ThreePhase)
	{
		// Only the 3-phase path can be current-controlled: current sensing exists solely on the DRV8316,
		// which is a 3-phase driver. Fall back to the q-only command so a mis-set configuration degrades
		// to voltage mode rather than driving something meaningless.
		ApplyTorque(vq, electricalAngle);
		return;
	}

	float sine, cosine;
	Trigonometry::FastSinCos(electricalAngle, sine, cosine);
	sine   /= 248.0f;
	cosine /= 248.0f;

	// Full inverse Park. The d = 0 case reduces to InversePark() above, which is the invariant that keeps
	// voltage mode and current mode commanding the same thing for the same q.
	const float alpha = (vd * cosine) - (vq * sine);
	const float beta  = (vd * sine)   + (vq * cosine);

	float duty_u, duty_v, duty_w;
	Svpwm(alpha, beta, duty_u, duty_v, duty_w);

	WritePhaseDuties(duty_u, duty_v, duty_w);
	lastDutyU = duty_u;
	lastDutyV = duty_v;
	lastDutyW = duty_w;
}

/*static*/ void FocController::MeasureDq(float ia, float ib, float ic, uint16_t electricalAngle,
		const SenseFrameCorrection& correction, float& id, float& iq) noexcept
{
	float sine, cosine;
	Trigonometry::FastSinCos(electricalAngle, sine, cosine);
	sine   /= 248.0f;
	cosine /= 248.0f;

	// Clarke, amplitude-preserving form. Deliberately uses all three measurements rather than the usual
	// two-phase shortcut (ialpha = ia, ibeta = (ia + 2*ib)/sqrt3, which assumes ia+ib+ic == 0). The three
	// current-sense amplifiers have independent zero offsets, so the measured sum carries a small DC bias
	// - around 40mA on the DRV8316 EVM - and the shortcut would fold all of it into ibeta as a fake
	// quadrature current. This form rejects it instead: a common offset d on all three phases contributes
	// (2d - d - d)/3 = 0 to ialpha and (d - d)/sqrt3 = 0 to ibeta. The third ADC reading is already being
	// taken, so using it is free.
	constexpr float OneOverSqrt3 = 0.5773502692f;
	float ialpha = (2.0f * ia - ib - ic) * (1.0f / 3.0f);
	float ibeta  = (ib - ic) * OneOverSqrt3;

	// Rotate (or reflect) the measured vector into the drive frame. Both cases are the standard 2x2:
	// a mirrored frame needs the reflection about half the offset angle, an aligned one the rotation by
	// minus the offset. Identity correction leaves ialpha/ibeta untouched, so hardware that is wired the
	// way the firmware assumes pays nothing for this.
	if (correction.mirrored)
	{
		const float a = (ialpha * correction.cosOffset) + (ibeta * correction.sinOffset);
		const float b = (ialpha * correction.sinOffset) - (ibeta * correction.cosOffset);
		ialpha = a; ibeta = b;
	}
	else if (correction.sinOffset != 0.0f)
	{
		const float a = (ialpha * correction.cosOffset) + (ibeta * correction.sinOffset);
		const float b = (ibeta * correction.cosOffset) - (ialpha * correction.sinOffset);
		ialpha = a; ibeta = b;
	}

	// Park - the exact inverse of InversePark() above; see the header for why that matters.
	id =  ialpha * cosine + ibeta * sine;
	iq = -ialpha * sine   + ibeta * cosine;
}

/*static*/ void FocController::Svpwm(float alpha, float beta,
		float& duty_u, float& duty_v, float& duty_w) noexcept
{
	// Standard 6-sector SVPWM using the reference vector's magnitude and angle directly.
	//
	// A previous version of this function derived the sector from the sign pattern of three
	// 120-degree-separated projections (Va/Vb/Vc) and switched on that pattern directly. That
	// was WRONG in two independent ways, both verified numerically (a full-revolution sweep at
	// 0.02-degree resolution, checking duty-cycle continuity): (1) the sign pattern does not
	// enumerate sectors in angular order - the pattern visits sectors in the order 1,3,2,6,4,5,
	// not 1,2,3,4,5,6, so switching on it directly applied the wrong sector's formula most of the
	// time; and (2) even after correcting for that mis-ordering, the six per-sector (t1,t2)
	// formulas were not mutually consistent at their shared boundaries (e.g. sector 2 and sector 3
	// disagreed by a large amount at the exact same physical angle). Together these caused duty
	// cycle jumps of up to ~100% of the full PWM range at six fixed electrical angles every
	// revolution, independent of commanded torque magnitude or direction - i.e. every time the
	// live commutation angle (in closed-loop mode) or the q-axis calibration sweep crossed one of
	// these six angles, the actual applied phase voltage bore no relation to the intended vector.
	// This is consistent with (and sufficient to fully explain) high current draw with no clean
	// torque-producing rotation, and with intermittent driver overcurrent faults specifically
	// during angle-sweeping operation (the calibration sweep, or any sustained rotation).
	//
	// This replacement computes the sector and in-sector angle directly from atan2f/sinf, which is
	// straightforward to verify by inspection and was confirmed continuous (residual ~1e-4, i.e.
	// floating-point step noise only) across a full revolution at multiple torque magnitudes.
	const float Vref = sqrtf(alpha*alpha + beta*beta);
	float theta = atan2f(beta, alpha);
	if (theta < 0.0f)
	{
		theta += TwoPi;
	}

	constexpr float sixtyDegrees = Pi / 3.0f;
	const int sector = min<int>((int)(theta / sixtyDegrees) + 1, 6);
	const float thetaInSector = theta - (float)(sector - 1) * sixtyDegrees;

	constexpr float sqrt3 = 1.7320508f;
	float t1 = Vref * sqrt3 * sinf(sixtyDegrees - thetaInSector);
	float t2 = Vref * sqrt3 * sinf(thetaInSector);

	const float sum = t1 + t2;
	if (sum > 1.0f)
	{
		t1 /= sum;
		t2 /= sum;
	}

	const float t0half = (1.0f - t1 - t2) * 0.5f;

	float ta, tb, tc;
	switch (sector)
	{
	case 1:  ta = t0half + t1 + t2; tb = t0half + t2;        tc = t0half;             break;
	case 2:  ta = t0half + t1;      tb = t0half + t1 + t2;   tc = t0half;             break;
	case 3:  ta = t0half;           tb = t0half + t1 + t2;   tc = t0half + t2;        break;
	case 4:  ta = t0half;           tb = t0half + t1;         tc = t0half + t1 + t2;  break;
	case 5:  ta = t0half + t2;      tb = t0half;              tc = t0half + t1 + t2;  break;
	default: ta = t0half + t1 + t2; tb = t0half;              tc = t0half + t1;       break;	// sector 6
	}

	duty_u = constrain<float>(ta, 0.0f, 1.0f);
	duty_v = constrain<float>(tb, 0.0f, 1.0f);
	duty_w = constrain<float>(tc, 0.0f, 1.0f);
}

void FocController::ApplyTorque3Phase(float torqueMagnitude, float sine, float cosine) noexcept
{
	float alpha, beta;
	InversePark(torqueMagnitude, sine, cosine, alpha, beta);

	float duty_u, duty_v, duty_w;
	Svpwm(alpha, beta, duty_u, duty_v, duty_w);

	WritePhaseDuties(duty_u, duty_v, duty_w);
	lastDutyU = duty_u;
	lastDutyV = duty_v;
	lastDutyW = duty_w;
}

void FocController::ApplyTorque2Phase(float torqueMagnitude, float sine, float cosine) noexcept
{
	// 2-phase inverse Park (d=0): coilA = q*cos(θ), coilB = q*sin(θ)
	const float coilA =  torqueMagnitude * cosine;	// ∈ [-1, 1]
	const float coilB =  torqueMagnitude * sine;	// ∈ [-1, 1]

	// Sign-magnitude mapping: one H-bridge input gets |duty|, other gets 0.
	// phaseU=IN1, phaseV=IN2 → coil A half-bridge
	// phaseW=IN3, in4Pin=IN4 → coil B half-bridge
	if (coilA >= 0.0f)
	{
		AnalogOut::Write(phaseU, coilA,   pwmFreq);
		AnalogOut::Write(phaseV, 0.0f,    pwmFreq);
	}
	else
	{
		AnalogOut::Write(phaseU, 0.0f,    pwmFreq);
		AnalogOut::Write(phaseV, -coilA,  pwmFreq);
	}

	if (coilB >= 0.0f)
	{
		AnalogOut::Write(phaseW, coilB,   pwmFreq);
		AnalogOut::Write(in4Pin, 0.0f,    pwmFreq);
	}
	else
	{
		AnalogOut::Write(phaseW, 0.0f,    pwmFreq);
		AnalogOut::Write(in4Pin, -coilB,  pwmFreq);
	}
}

void FocController::ApplyTorqueHybrid(float torqueMagnitude, float sine, float cosine) noexcept
{
	// 2-phase inverse Park — same math as stepper FOC but output via 3 PWM pins
	const float Ua = -torqueMagnitude * sine;	// coil A voltage ∈ [-1, 1]
	const float Ub =  torqueMagnitude * cosine;	// coil B voltage ∈ [-1, 1]

	// SVPWM zero-sequence injection: shifts all three duties into [0, 1]
	// while maximising coil voltage swing vs. fixed-centre SinePWM.
	const float Umin = min(min(Ua, Ub), 0.0f);
	const float Umax = max(max(Ua, Ub), 0.0f);
	const float Vo   = -(Umin + Umax) * 0.5f + 0.5f;	// synthetic C (midpoint) offset

	lastDutyU = constrain<float>(Ua + Vo, 0.0f, 1.0f);	// coil A
	lastDutyV = constrain<float>(Ub + Vo, 0.0f, 1.0f);	// coil B
	lastDutyW = constrain<float>(Vo,      0.0f, 1.0f);	// motor midpoint
	WritePhaseDuties(lastDutyU, lastDutyV, lastDutyW);
}

void FocController::ApplyTorque(float torqueMagnitude, uint16_t electricalAngle) noexcept
{
	float sine, cosine;
	Trigonometry::FastSinCos(electricalAngle, sine, cosine);
	// FastSinCos outputs in [-248, 248]; normalise to [-1, 1]
	sine   /= 248.0f;
	cosine /= 248.0f;

	switch (outputMode)
	{
	case FocOutputMode::ThreePhase:
		ApplyTorque3Phase(torqueMagnitude, sine, cosine);
		break;
	case FocOutputMode::TwoPhase4Pwm:
		ApplyTorque2Phase(torqueMagnitude, sine, cosine);
		break;
	case FocOutputMode::HybridStepper:
		ApplyTorqueHybrid(torqueMagnitude, sine, cosine);
		break;
	}
}
