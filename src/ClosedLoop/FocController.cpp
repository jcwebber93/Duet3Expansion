/*
 * FocController.cpp
 */

#include "FocController.h"
#include <RepRapFirmware.h>
#include <AnalogOut.h>

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

void FocController::Init(PwmFrequency freq) noexcept
{
	pwmFreq = freq;

	if (outputMode == FocOutputMode::TwoPhase4Pwm)
	{
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
	else
	{
		SetPinFunction(phaseU, fnU);
		SetPinFunction(phaseV, fnV);
		SetPinFunction(phaseW, fnW);
		// 50% duty = zero average voltage across all windings
		AnalogOut::Write(phaseU, 0.5f, pwmFreq);
		AnalogOut::Write(phaseV, 0.5f, pwmFreq);
		AnalogOut::Write(phaseW, 0.5f, pwmFreq);
	}
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
		AnalogOut::Write(phaseU, 0.5f, pwmFreq);
		AnalogOut::Write(phaseV, 0.5f, pwmFreq);
		AnalogOut::Write(phaseW, 0.5f, pwmFreq);
	}
}

/*static*/ void FocController::InversePark(float q, float sine, float cosine,
		float& alpha, float& beta) noexcept
{
	// d = 0 (voltage mode): alpha = -q*sin(θ), beta = q*cos(θ)
	alpha = -q * sine;
	beta  =  q * cosine;
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

	AnalogOut::Write(phaseU, duty_u, pwmFreq);
	AnalogOut::Write(phaseV, duty_v, pwmFreq);
	AnalogOut::Write(phaseW, duty_w, pwmFreq);
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

	lastDutyU = constrain<float>(Ua + Vo, 0.0f, 1.0f);
	lastDutyV = constrain<float>(Ub + Vo, 0.0f, 1.0f);
	lastDutyW = constrain<float>(Vo,      0.0f, 1.0f);
	AnalogOut::Write(phaseU, lastDutyU, pwmFreq);	// coil A
	AnalogOut::Write(phaseV, lastDutyV, pwmFreq);	// coil B
	AnalogOut::Write(phaseW, lastDutyW, pwmFreq);	// motor midpoint
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
