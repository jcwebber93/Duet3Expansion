/*
 * FocCurrentSense.h — inline phase-current measurement for FOC drives
 *
 * Reads the DRV8316's three current-sense amplifier outputs on ADC0.
 *
 * ADC0 is taken over entirely by this module rather than shared with CoreN2G's AnalogIn service. That
 * service is a millisecond-rate round-robin configured for thermistors - 16-bit with 64x hardware
 * averaging, well over 100us per reading against a 50us PWM period - so it cannot be used for
 * PWM-synchronous sampling. Taking the device is cheap on this board: AnalogIn only touches ADC0
 * hardware when a channel is first enabled (see AdcClass::InternalEnableChannel -> ReInit), and once the
 * MCU temperature sensors are moved to ADC1 nothing enables a channel on ADC0 at all. The clocks are
 * still set up for us by AnalogIn::Init().
 *
 * Sampling is triggered by the TCC0 overflow event, which lands at the centre of the interval where all
 * three low-side FETs conduct - the quiet point in the PWM period, and the only instant at which the
 * CSA outputs are settled. Polled reads (ReadRaw) remain available but sample at an arbitrary point in
 * the carrier and will contain switching noise; they are for wiring verification via M122, not control.
 *
 * Verified on TI DRV8316 EVM hardware: zero offsets within ~20mA of nominal, and phase currents that
 * sum to zero and track commanded torque during motion.
 */

#ifndef SRC_CLOSEDLOOP_FOCCURRENTSENSE_H_
#define SRC_CLOSEDLOOP_FOCCURRENTSENSE_H_

#include <RepRapFirmware.h>

#if SUPPORT_FOC && SUPPORT_DRV8316_SPI

namespace FocCurrentSense
{
	// Claim and configure ADC0. Safe to call more than once. Returns false if the ADC did not respond.
	bool Init() noexcept;

	bool IsInitialised() noexcept;

	// Number of sense phases (A, B, C).
	static constexpr unsigned int NumPhases = 3;

	// Take one polled conversion on the given phase (0..2) and return the raw 12-bit count.
	// Blocking, roughly 3us. Not synchronised to the PWM carrier - see the file header.
	uint16_t ReadRaw(unsigned int phase) noexcept;

	// Route the TCC0 overflow event to the ADC's start input, so conversions happen at the centre of the
	// window where all three low-side FETs conduct. Call once the PWM carrier is running.
	void StartSynchronisedSampling() noexcept;

	bool IsSynchronised() noexcept;

	// Service one control tick: pick up a completed three-phase scan, if one has finished, and re-arm.
	// Cheap (a flag read, three subtractions and the re-arm) and safe to call when not synchronised.
	//
	// The scan itself is hardware-driven - see StartSynchronisedSampling() - so this only collects the
	// result. Polling rather than an interrupt is forced: CoreN2G defines ADC0_1_Handler, the RESRDY
	// vector, as a strong symbol. That costs nothing here, because the thing that matters is WHEN the
	// conversion is taken, and that comes from the carrier event regardless of when we read it.
	void Poll() noexcept;

	// Poll until one fresh conversion has been collected for every phase, so GetPhaseCurrents() returns a
	// set captured within a few hundred microseconds of each other rather than up to NumPhases control
	// ticks apart. Blocks; strictly for calibration and diagnostic paths that run outside the control
	// loop. Returns false if the timeout expired first, which means conversions are not arriving.
	bool RefreshAllPhases(uint32_t timeoutUs) noexcept;

	// Latest measured phase currents in amps, from one hardware scan: the three readings are taken one
	// PWM period apart, so the set spans ~150us and is internally consistent in the sense that all three
	// come from the same scan. Sets that fail a Kirchhoff check are discarded rather than published.
	//
	// Not simultaneous, though - see StartSynchronisedSampling() for why that needs hardware this board
	// does not have. Over 150us the rotor barely turns, but a fast-changing torque command does move, so
	// a current loop closed around these should keep its bandwidth well below 1/150us.
	void GetPhaseCurrents(float& ia, float& ib, float& ic) noexcept;

	// Convert a raw count to amps using the DRV8316's CSA gain and that phase's measured zero offset.
	float RawToAmps(unsigned int phase, uint16_t raw) noexcept;

	// Average several conversions per phase with the output stage idle, to establish the zero-current
	// point. The caller must ensure the motor is not being driven. Returns false if any phase settled
	// implausibly far from the expected mid-scale, which indicates a wiring or reference problem.
	bool CalibrateZeroOffset(const StringRef& reply) noexcept;

	// Raw counts corresponding to zero current, one per phase, from the last calibration.
	uint16_t GetZeroOffset(unsigned int phase) noexcept;

	// True once CalibrateZeroOffset() has succeeded. Until then readings are referenced to the nominal
	// mid-scale point rather than a measured one, so they carry an unknown offset of tens of milliamps.
	bool IsCalibrated() noexcept;

	// True if a complete, self-consistent, unsaturated scan has been collected recently. GetPhaseCurrents()
	// returns the last accepted scan whether or not this holds, which is right for telemetry and WRONG for
	// feedback: any control loop must check this and stop feeding on the values when it goes false.
	// Saturating an amplifier makes every subsequent scan unusable, so without this check the feedback
	// freezes at the reading that caused the saturation and the loop holds the fault indefinitely.
	bool IsMeasurementFresh() noexcept;

	void AppendDiagnostics(const StringRef& reply) noexcept;
}

#endif

#endif /* SRC_CLOSEDLOOP_FOCCURRENTSENSE_H_ */
