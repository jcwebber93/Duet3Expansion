/*
 * DRV8316.cpp
 */

#include "DRV8316.h"

// SPI: 1 MHz, Mode 1 (CPOL=0 CPHA=1), CS active low
DRV8316::DRV8316(SharedSpiDevice& spi, Pin csPin) noexcept
	: spiClient(spi, 1000000u, SpiMode::mode1, csPin, false)
{
}

// Build a 16-bit frame: [15]=R/W, [14:9]=addr, [8]=parity, [7:0]=data.
// Even parity: the parity bit is set so that the total number of 1-bits in the word is even.
/*static*/ uint16_t DRV8316::BuildFrame(bool read, uint8_t addr, uint8_t data) noexcept
{
	// Assemble without parity first
	uint16_t frame = (uint16_t)(((read ? 0x80u : 0x00u) | ((addr & 0x3Fu) << 1)) << 8) | data;
	// Count set bits in all 15 non-parity positions (bit 8 is parity, currently 0)
	uint16_t tmp = frame;
	unsigned int ones = 0;
	while (tmp)
	{
		ones += tmp & 1u;
		tmp >>= 1;
	}
	// If count is odd, set parity bit (bit 8) to make it even
	if (ones & 1u)
	{
		frame |= 0x0100u;
	}
	return frame;
}

uint8_t DRV8316::ReadReg(uint8_t addr) noexcept
{
	const uint16_t frame = BuildFrame(true, addr, 0x00);
	uint8_t tx[2] = { (uint8_t)(frame >> 8), (uint8_t)(frame & 0xFFu) };
	uint8_t rx[2] = { 0, 0 };
	if (spiClient.Select())
	{
		spiClient.TransceivePacket(tx, rx, 2);
		spiClient.Deselect();
	}
	delayMicroseconds(1);
	return rx[1];	// [7:0] = register data; [15:8] = IC_Status echo (ignored here)
}

void DRV8316::WriteReg(uint8_t addr, uint8_t val) noexcept
{
	const uint16_t frame = BuildFrame(false, addr, val);
	uint8_t tx[2] = { (uint8_t)(frame >> 8), (uint8_t)(frame & 0xFFu) };
	if (spiClient.Select())
	{
		spiClient.TransceivePacket(tx, nullptr, 2);
		spiClient.Deselect();
	}
	delayMicroseconds(1);
}

bool DRV8316::Init() noexcept
{
	// Step 1: unlock control registers (Control_1 REG_LOCK = 0b011)
	WriteReg(RegControl1, RegLockUnlocked);

	// Step 2: set 3PWM mode and clear any power-on faults
	WriteReg(RegControl2, Ctrl2Pwm3Mode | Ctrl2ClrFlt);

	// Step 3: verify comms by reading IC_Status — 0xFF indicates a dead bus
	lastIcStatus = ReadReg(RegIcStatus);
	spiPresent = (lastIcStatus != 0xFFu);
	return spiPresent;
}

void DRV8316::PollFaults() noexcept
{
	lastIcStatus = ReadReg(RegIcStatus);
	if (lastIcStatus & 0x01u)	// FAULT bit set — read detail registers
	{
		lastStatus1 = ReadReg(RegStatus1);
		lastStatus2 = ReadReg(RegStatus2);
	}
}

void DRV8316::ClearFaults() noexcept
{
	// Read-modify-write: preserve other Control_2 bits, set CLR_FLT
	const uint8_t ctrl2 = ReadReg(RegControl2);
	WriteReg(RegControl2, ctrl2 | Ctrl2ClrFlt);
	// Hardware clears CLR_FLT automatically after the write; no need to clear it again
	lastIcStatus = 0;
	lastStatus1  = 0;
	lastStatus2  = 0;
}
