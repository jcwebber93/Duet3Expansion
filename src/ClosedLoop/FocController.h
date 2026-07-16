/*
 * FocController.h
 *
 * Field Oriented Control (FOC) output stage for BLDC/PMSM and stepper motors.
 * Voltage-mode only (first pass): d=0, q=torque_magnitude, no current sensing.
 *
 * Three output modes:
 *   ThreePhase   — 3PWM BLDC/PMSM via SVPWM (EncoderType::bldc)
 *   TwoPhase4Pwm — 4PWM 2-phase stepper via sign-magnitude H-bridge (EncoderType::stepperFoc)
 *   HybridStepper — 3PWM hybrid stepper: 2-phase cos/sin + synthetic C midpoint (EncoderType::hybridStepperFoc)
 *
 * PWM pins do not need to share a TCC instance for voltage-mode FOC.
 * Center-aligned single-TCC mode is deferred until current-sensing support is added.
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

	// Drive all phases to neutral (zero average voltage, motor coasts).
	void Coast() noexcept;

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
