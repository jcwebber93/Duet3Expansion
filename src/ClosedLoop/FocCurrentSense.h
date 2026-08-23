/*
 * FocCurrentSense.h — inline phase-current measurement for FOC drives
 *
 * Reads the DRV8316's three current-sense amplifier outputs on ADC0, which this module owns outright
 * rather than sharing with CoreN2G's AnalogIn service. Conversions are triggered by the TCC0 overflow
 * event so they land at the quiet point of the PWM carrier. Polled reads (ReadRaw) sample at an
 * arbitrary point in the carrier and carry switching noise; they are for wiring verification, not
 * control.
 *
 * Background: docs/foc-current-sense.md
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

	// Service one control tick: collect a completed three-phase scan, if one has finished, and re-arm.
	// Cheap, and safe to call when not synchronised.
	//
	// Polled, not interrupt-driven: CoreN2G defines ADC0_1_Handler (the RESRDY vector) as a strong symbol.
	void Poll() noexcept;

	// Poll until a fresh scan has been collected. BLOCKS - strictly for calibration and diagnostic paths
	// that run outside the control loop. Returns false on timeout, meaning conversions are not arriving.
	bool RefreshAllPhases(uint32_t timeoutUs) noexcept;

	// Latest phase currents in amps, all three from one hardware scan. Readings are one PWM period apart,
	// so the set spans ~150us - NOT simultaneous. Keep any current loop's bandwidth well below 1/150us.
	// Scans failing a Kirchhoff check are discarded rather than published.
	void GetPhaseCurrents(float& ia, float& ib, float& ic) noexcept;

	// Convert a raw count to amps using the DRV8316's CSA gain and that phase's measured zero offset.
	float RawToAmps(unsigned int phase, uint16_t raw) noexcept;

	// Measure the zero-current point by averaging conversions with the output stage idle. THE CALLER MUST
	// ENSURE THE MOTOR IS NOT BEING DRIVEN. Returns false if any phase settled implausibly far from
	// mid-scale, which means a wiring or reference problem.
	bool CalibrateZeroOffset(const StringRef& reply) noexcept;

	// Raw counts corresponding to zero current, one per phase, from the last calibration.
	uint16_t GetZeroOffset(unsigned int phase) noexcept;

	// True once CalibrateZeroOffset() has succeeded. Until then readings are referenced to nominal
	// mid-scale and carry an unknown offset of tens of milliamps.
	bool IsCalibrated() noexcept;

	// True if a complete, unsaturated, self-consistent scan has been collected recently.
	//
	// ANY CONTROL LOOP MUST CHECK THIS and stop using GetPhaseCurrents() when it goes false - that keeps
	// returning the last accepted scan, which is right for telemetry and freezes a feedback loop.
	// Background: docs/foc-current-sense.md#staleness
	bool IsMeasurementFresh() noexcept;

	// True when the amplifiers are clipping - an overcurrent report, not an absence of data.
	bool IsSaturated() noexcept;

	void AppendDiagnostics(const StringRef& reply) noexcept;
}

#endif

#endif /* SRC_CLOSEDLOOP_FOCCURRENTSENSE_H_ */
