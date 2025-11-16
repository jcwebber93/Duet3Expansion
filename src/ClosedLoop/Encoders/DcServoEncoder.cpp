/*
 * DcServoEncoder.cpp
 *
 *  Created on: 14 Nov 2025
 *      Author: JCW + help
 */

#include <RepRapFirmware.h>

#if SUPPORT_DCSERVO

#include "DcServoEncoder.h"
#include "QuadratureEncoderPdec.h"
#include <hri_mclk_e54.h>
#include <cmath>

DcServoEncoder::DcServoEncoder(uint32_t p_countsPerRev, uint32_t p_stepsPerRev) noexcept
	: Encoder(1.0, p_stepsPerRev), lastCount(0), counterHigh(0), pulsesPerRev(p_countsPerRev)
{
}

// Initialise the encoder and enable it if successful.
GCodeResult DcServoEncoder::Init(const StringRef& reply) noexcept
{
	// Set up the clocks
	QuadratureEncoderPdec::SetupClocks();

	PDEC->CTRLA.bit.ENABLE = 0;
	while (PDEC->SYNCBUSY.bit.ENABLE) { }
	PDEC->CTRLA.bit.SWRST = 1;
	while (PDEC->SYNCBUSY.bit.SWRST) { }

	for (Pin p : PositionDecoderPins)
	{
		SetPinFunction(p, PositionDecoderPinFunction);
	}

	uint32_t ctrla = PDEC_CTRLA_MODE_QDEC | PDEC_CTRLA_CONF_X4
					| PDEC_CTRLA_PINEN0 | PDEC_CTRLA_PINEN1
					| PDEC_CTRLA_ANGULAR(7);
	PDEC->CTRLA.reg = ctrla;

	Enable();
	return GCodeResult::ok;
}

void DcServoEncoder::Enable() noexcept
{
	SetPosition(0);
	PDEC->CTRLA.bit.ENABLE = 1;
	while (PDEC->SYNCBUSY.bit.ENABLE) { }
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_START;
	while (PDEC->SYNCBUSY.bit.CTRLB) { }
}

void DcServoEncoder::Disable() noexcept
{
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_STOP;
	while (PDEC->SYNCBUSY.bit.CTRLB) { }
	PDEC->CTRLA.bit.ENABLE = 0;
	while (PDEC->SYNCBUSY.bit.ENABLE) { }
}

bool DcServoEncoder::TakeReading() noexcept
{
	bool err;
	currentCount = GetRelativePosition(err);
	// currentPhasePosition is not used for a DC servo, but clear it for safety.
	currentPhasePosition = 0;
	return err;
}

void DcServoEncoder::ClearFullRevs() noexcept
{
	counterHigh = (lastCount & 0x8000) ? 0xFFFF : 0;
	(void)TakeReading();
}

void DcServoEncoder::AppendDiagnostics(const StringRef &reply) noexcept
{
	reply.catf("DC Servo Encoder, raw count %" PRIi32, currentCount);
}

void DcServoEncoder::AppendStatus(const StringRef& reply) noexcept
{
	reply.lcatf("Quadrature encoder PPR: %" PRIu32 " (%" PRIu32 " CPR)", pulsesPerRev, pulsesPerRev * 4);
}

// Get the current position relative to the starting position
int32_t DcServoEncoder::GetRelativePosition(bool& error) noexcept
{
	PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_READSYNC;
	while (PDEC->SYNCBUSY.reg & (PDEC_SYNCBUSY_CTRLB | PDEC_SYNCBUSY_COUNT)) { }
	const uint16_t count = PDEC->COUNT.reg;

	// Handle wrap around of the high position bits
	const uint16_t currentHighBits = count >> 14;
	const uint16_t lastHighBits = lastCount >> 14;
	if (currentHighBits == 3 && lastHighBits == 0)
	{
		--counterHigh;
	}
	else if (currentHighBits == 0 && lastHighBits == 3)
	{
		++counterHigh;
	}

	lastCount = count;

	error = false;
	return (int32_t)((counterHigh << 16) | count);
}

// Set the position to the 32 bit signed value 'position'
void DcServoEncoder::SetPosition(int32_t position) noexcept
{
	while (PDEC->SYNCBUSY.bit.STATUS) { }
	const bool stopped = PDEC->STATUS.bit.STOP;
	if (!stopped)
	{
		PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_STOP;
		while (PDEC->CTRLBSET.bit.CMD != 0) { }
	}

	PDEC->COUNT.reg = lastCount = (uint16_t)position;
	counterHigh = (uint32_t)position >> 16;

	if (!stopped)
	{
		PDEC->CTRLBSET.reg = PDEC_CTRLBSET_CMD_START;
		while (PDEC->CTRLBSET.bit.CMD != 0) { }
	}
}
#endif
