/*
 * DcServoEncoder.h
 *
 *  Created on: 14 Nov 2025
 *      Author: Gemini
 */

#ifndef SRC_CLOSEDLOOP_ENCODERS_DCSERVOENCODER_H_
#define SRC_CLOSEDLOOP_ENCODERS_DCSERVOENCODER_H_

#include "Encoder.h"

#include <General/FreelistManager.h>

// Class to use the Position Decoder peripheral (PDEC) as a quadrature decoder for a DC servo motor
class DcServoEncoder : public Encoder
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<DcServoEncoder>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<DcServoEncoder>(p); }

	DcServoEncoder(uint32_t p_countsPerRev, uint32_t p_stepsPerRev) noexcept;
	~DcServoEncoder() { DcServoEncoder::Disable(); }

	EncoderType GetType() const noexcept override { return EncoderType::dcServo; }
	GCodeResult Init(const StringRef& reply) noexcept override;
	void Enable() noexcept override;
	void Disable() noexcept override;
	bool TakeReading() noexcept override;
	void ClearFullRevs() noexcept override;
	void AppendDiagnostics(const StringRef& reply) noexcept override;
	void AppendStatus(const StringRef& reply) noexcept override;

	// The following functions are stepper-specific and not applicable to a DC servo, so we provide stub implementations.
	void SetKnownPhaseAtCurrentCount(uint32_t phase) noexcept override { }
	void SetTuningBackwards(bool backwards) noexcept override { }
	void SetCalibrationBackwards(bool backwards) noexcept override { }
	bool UsesCalibration() const noexcept override { return false; }
	bool UsesBasicTuning() const noexcept override { return false; }
	void SetForwardTuningResults(float slope, float xMean, float yMean) noexcept override { }
	void SetReverseTuningResults(float slope, float xMean, float yMean) noexcept override { }
	TuningErrors ProcessTuningData(bool isLinearEncoder) noexcept override { return 0; }
	void ClearDataCollection(size_t p_numDataPoints) noexcept override { }
	void RecordDataPoint(size_t index, int32_t data, bool backwards) noexcept override { }
	void LoadLUT(TuningErrors& tuningNeeded) noexcept override { }
	void ClearLUT() noexcept override { }
	void ScrubLUT() noexcept override { }
	TuningErrors Calibrate(bool store) noexcept override { return TuningError::SystemError; }
	void AppendLUTCorrections(const StringRef& reply) const noexcept override { }
	void AppendCalibrationErrors(const StringRef& reply) const noexcept override { }

private:
	// Get the current position relative to the starting position
	int32_t GetRelativePosition(bool& error) noexcept;
	void SetPosition(int32_t position) noexcept;

	uint16_t lastCount;
	uint32_t counterHigh;
	uint32_t pulsesPerRev;
};

#endif /* SRC_CLOSEDLOOP_ENCODERS_DCSERVOENCODER_H_ */