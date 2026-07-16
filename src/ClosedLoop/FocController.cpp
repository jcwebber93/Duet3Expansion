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
	// Standard 6-sector SVPWM using reference voltage projections.
	const float Va =  alpha;
	const float Vb = (-alpha + 1.7320508f * beta) * 0.5f;
	const float Vc = (-alpha - 1.7320508f * beta) * 0.5f;

	const int sector = ((Va > 0.0f) ? 1 : 0)
	                 + ((Vb > 0.0f) ? 2 : 0)
	                 + ((Vc > 0.0f) ? 4 : 0);

	float t1, t2;
	switch (sector)
	{
	case 1:  t1 =  Va; t2 = -Vc; break;
	case 2:  t1 =  Vb; t2 =  Va; break;
	case 3:  t1 = -Vc; t2 =  Vb; break;
	case 4:  t1 = -Va; t2 =  Vc; break;
	case 5:  t1 =  Vc; t2 = -Vb; break;
	case 6:  t1 = -Vb; t2 = -Va; break;
	default: t1 = 0.0f; t2 = 0.0f; break;
	}

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
	case 6:  ta = t0half + t1 + t2; tb = t0half;              tc = t0half + t1;       break;
	default: ta = 0.5f;             tb = 0.5f;                tc = 0.5f;              break;
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
