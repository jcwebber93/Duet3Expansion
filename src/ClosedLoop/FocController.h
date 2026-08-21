/*
 * FocController.h
 *
 * Field Oriented Control (FOC) output stage for BLDC/PMSM and stepper motors.
 *
 * Takes either a torque magnitude (ApplyTorque, d = 0) or a d/q voltage vector (ApplyDqVoltage), plus an
 * electrical angle, and drives the phase PWMs.
 *
 * Three output modes:
 *   ThreePhase   — 3PWM BLDC/PMSM via SVPWM (EncoderType::bldc)
 *   TwoPhase4Pwm — 4PWM 2-phase stepper via sign-magnitude H-bridge (EncoderType::stepperFoc)
 *   HybridStepper — 3PWM hybrid stepper: 2-phase cos/sin + synthetic C midpoint (EncoderType::hybridStepperFoc)
 *
 * Boards defining FOC_CENTRE_ALIGNED_PWM get a dual-slope carrier driven from this class, which inline
 * current sensing requires. Boards with phases on independent TCCs fall back to AnalogOut::Write(),
 * which works for voltage mode but cannot support synchronous sampling.
 */

#ifndef SRC_CLOSEDLOOP_FOCCONTROLLER_H_
#define SRC_CLOSEDLOOP_FOCCONTROLLER_H_

#include "RepRapFirmware.h"
#include "ClosedLoop/Trigonometry.h"

enum class FocOutputMode : uint8_t
{
	ThreePhase      = 0,	// BLDC: 3 PWM pins, full SVPWM
	TwoPhase4Pwm    = 1,	// Stepper FOC: 4 PWM pins, sign-magnitude per H-bridge coil
	HybridStepper   = 2,	// Hybrid stepper: 3 PWM pins (same hw as ThreePhase), 2-phase + synthetic C offset
};

class FocController
{
public:
	// Constructor for 3-phase BLDC (ThreePhase) or hybrid stepper (HybridStepper)
	FocController(Pin u, Pin v, Pin w,
				  GpioPinFunction uFn, GpioPinFunction vFn, GpioPinFunction wFn,
				  FocOutputMode mode = FocOutputMode::ThreePhase) noexcept;

	// Constructor for 4PWM 2-phase stepper (TwoPhase4Pwm)
	FocController(Pin in1, Pin in2, Pin in3, Pin in4,
				  GpioPinFunction fn1, GpioPinFunction fn2,
				  GpioPinFunction fn3, GpioPinFunction fn4,
				  Pin ena = NoPin, Pin enb = NoPin) noexcept;

	void Init(PwmFrequency freq) noexcept;

	// Apply a torque demand in voltage mode.
	// torqueMagnitude is in [-1, +1] (fraction of bus voltage).
	// electricalAngle is in [0, 4095] matching Trigonometry::FastSinCos().
	void ApplyTorque(float torqueMagnitude, uint16_t electricalAngle) noexcept;

	// Apply a d/q voltage vector, for current-mode control. Both are fractions of the bus voltage, and
	// the CALLER must keep the magnitude within its voltage budget - see ClosedLoop's circular limit.
	// ApplyTorque() above is the d = 0 case, still used by every drive without current sensing.
	void ApplyDqVoltage(float vd, float vq, uint16_t electricalAngle) noexcept;

	// Drive all phases to neutral (zero average voltage, motor coasts).
	void Coast() noexcept;

	// How the current-sense channels sit relative to the drive phases, measured by the alignment sweep.
	// Identity means correctly wired hardware; anything else compensates for channels that do not line up
	// with the phases they measure.
	//
	// The caller snaps the offset to a multiple of 60 degrees - only those are physically reachable. What
	// is left over is the winding's load angle, which is real and must NOT be calibrated out.
	struct SenseFrameCorrection
	{
		float cosOffset = 1.0f;			// cos and sin of the snapped offset
		float sinOffset = 0.0f;
		bool mirrored = false;			// sense frame rotates opposite to the drive frame
	};

	// Measured phase currents → rotor reference frame (Clarke then Park).
	//
	// electricalAngle uses the same convention as ApplyTorque(), and this is the exact inverse of
	// InversePark(), so iq is directly comparable with the commanded value and id measures whether the
	// commutation angle is right - a correctly aligned drive leaves id near zero.
	static void MeasureDq(float ia, float ib, float ic, uint16_t electricalAngle,
						  const SenseFrameCorrection& correction, float& id, float& iq) noexcept;

private:
	// Inverse Park transform: (d=0, q) → (alpha, beta).
	// sine/cosine must be in [-1, +1] (pre-divided by 248).
	static void InversePark(float q, float sine, float cosine, float& alpha, float& beta) noexcept;

	// 3-phase SVPWM — outputs duty_u/v/w in [0, 1].
	static void Svpwm(float alpha, float beta, float& duty_u, float& duty_v, float& duty_w) noexcept;

	// Output stage dispatchers
	void ApplyTorque3Phase(float torqueMagnitude, float sine, float cosine) noexcept;
	void ApplyTorque2Phase(float torqueMagnitude, float sine, float cosine) noexcept;
	void ApplyTorqueHybrid(float torqueMagnitude, float sine, float cosine) noexcept;

	// Write the three phase duty cycles (each 0..1). Routes to the centre-aligned TCC path where the
	// board provides one, otherwise to AnalogOut::Write(). Single choke point so the two 3-pin output
	// modes cannot diverge in how they reach the timer.
	void WritePhaseDuties(float dutyU, float dutyV, float dutyW) noexcept;

#if defined(FOC_CENTRE_ALIGNED_PWM) && FOC_CENTRE_ALIGNED_PWM
	// Configure the shared TCC for dual-slope (centre-aligned) PWM and emit an overflow event at the
	// carrier centre for the current-sense ADC trigger. See FOC_CENTRE_ALIGNED_PWM in the board config.
	void InitCentreAlignedPwm() noexcept;

	uint32_t pwmPeriod = 0;			// TCC PER value; in dual-slope mode one carrier period is 2*PER ticks
#endif

	// Pins for the 3-pin (ThreePhase / HybridStepper) path.
	// Also used as IN1/IN2/IN3 for the 4-pin (TwoPhase4Pwm) path.
	Pin phaseU, phaseV, phaseW;
	GpioPinFunction fnU, fnV, fnW;

	// Extra pin and function for IN4 in TwoPhase4Pwm mode.
	Pin in4Pin;
	GpioPinFunction fnIn4;

	PwmFrequency pwmFreq = 20000;
	FocOutputMode outputMode;

public:
	float lastDutyU = 0.5f;
	float lastDutyV = 0.5f;
	float lastDutyW = 0.5f;
};

#endif /* SRC_CLOSEDLOOP_FOCCONTROLLER_H_ */
