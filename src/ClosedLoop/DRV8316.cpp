/*
 * DRV8316.cpp
 */

#include "DRV8316.h"

// SPI: 1 MHz, Mode 1 (CPOL=0 CPHA=1), CS active low. Matches the reference implementation's
// SPISettings(1000000, MSBFIRST, SPI_MODE1).
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

// One 16-bit transaction. The response is always [15:8] = IC_Status, [7:0] = register data - on writes
// as well as reads - so latch the status every time and return the data byte.
uint8_t DRV8316::Transfer(uint16_t frame) noexcept
{
	uint8_t tx[2] = { (uint8_t)(frame >> 8), (uint8_t)(frame & 0xFFu) };
	uint8_t rx[2] = { 0, 0 };
	if (spiClient.Select())
	{
		spiClient.TransceivePacket(tx, rx, 2);
		spiClient.Deselect();
		lastIcStatus = rx[0];
	}
	delayMicroseconds(1);				// device requires at least 400ns between transactions
	return rx[1];
}

uint8_t DRV8316::ReadReg(uint8_t addr) noexcept
{
	return Transfer(BuildFrame(true, addr, 0x00));
}

void DRV8316::WriteReg(uint8_t addr, uint8_t val) noexcept
{
	(void)Transfer(BuildFrame(false, addr, val));
}

// Read-modify-write with verification. Blind writes are wrong here: every control register packs
// several unrelated settings, so writing only the field you care about zeroes the rest. The previous
// version of Init() wrote Control_2 blind and silently reset SLEW and SDO_MODE to 0 as a side effect.
bool DRV8316::UpdateReg(uint8_t addr, uint8_t mask, uint8_t val) noexcept
{
	const uint8_t current = ReadReg(addr);
	const uint8_t wanted = (uint8_t)((current & (uint8_t)~mask) | (val & mask));
	WriteReg(addr, wanted);
	const uint8_t readBack = ReadReg(addr);
	return (readBack & mask) == (val & mask);
}

bool DRV8316::Init() noexcept
{
	// Verify comms first - 0xFF on every bit indicates a dead bus (MISO stuck high, no device).
	lastIcStatus = 0;
	const uint8_t status = ReadReg(RegIcStatus);
	spiPresent = (status != 0xFFu);
	configVerified = false;
	if (!spiPresent)
	{
		return false;
	}

	// Unlock the control registers. REG_LOCK is itself in a control register, so this has to come first.
	if (!UpdateReg(RegControl1, Ctrl1RegLockMask, Ctrl1RegLockUnlocked))
	{
		return false;
	}

	// Configuration. These are hard-coded rather than exposed as gcode parameters; each choice is
	// recorded with its reasoning so it can be revisited deliberately.
	bool ok = true;

	// 3PWM mode - this firmware drives three half-bridges from three PWM outputs (see FocController).
	// SLEW 50 V/us is a compromise: slower than the 150/200 V/us options so switching-edge ringing
	// settles well before the current-sense sampling instant, but fast enough to keep switching loss
	// and dead-time distortion low. CLR_FLT is deliberately not set here; Init() should not mask a
	// power-on fault that the caller ought to see.
	ok = UpdateReg(RegControl2, (uint8_t)(Ctrl2PwmModeMask | Ctrl2SlewMask),
					(uint8_t)(Ctrl2PwmMode3Pwm | Ctrl2Slew50Vus)) && ok;

	// Over-current protection: latched rather than auto-retry, so a genuine fault is reported once and
	// stays reported instead of being silently retried behind our back. 16 A threshold (OCP_LVL = 0) is
	// the lower of the two options and ample for a NEMA17-class motor. 1.1 us deglitch avoids nuisance
	// trips on switching transients. Cycle-by-cycle clearing is disabled for the same reason as
	// auto-retry. DRV_OFF is explicitly cleared so the output stage is enabled.
	ok = UpdateReg(RegControl4,
					(uint8_t)(Ctrl4OcpModeMask | Ctrl4OcpLvl | Ctrl4OcpDegMask | Ctrl4OcpCbc | Ctrl4DrvOff),
					(uint8_t)(Ctrl4OcpModeLatched | Ctrl4OcpDeg1us1)) && ok;

	// Current-sense amplifier gain. The CSA output is bidirectional, centred on VREF/2 and swinging over
	// the full 0..VREF range - note that is the DRV8316's own VREF, not the MCU's ADC reference. On the
	// TI EVM with its default VREF select of 3.0 V that is 1.5 V +-1.5 V, so 0.25 V/A gives a +-6.0 A
	// full scale with the zero-current point at 1.5 V. At 12-bit that is roughly 3 mA per LSB, ample for
	// a NEMA17-class motor drawing a couple of amps, with headroom for transients.
	//
	// Note the CSA saturates at 6 A while OCP is set at 16 A. That is not a safety gap - OCP is an
	// independent comparator on the FET current, not derived from the CSA - but it does mean a genuine
	// overcurrent reads as a clipped 6 A rather than its true value. Raise the gain (0.375 V/A -> +-4 A)
	// for better resolution on a small motor, or lower it (0.15 V/A -> +-10 A) to see more of a fault.
	// Must stay in step with CsaGainVoltsPerAmp in the header.
	//
	// Active synchronous and asynchronous rectification are both disabled: they change how current
	// recirculates during the off period, which would otherwise complicate interpreting the sampled
	// phase current during bring-up.
	ok = UpdateReg(RegControl5, (uint8_t)(Ctrl5CsaGainMask | Ctrl5EnAsr | Ctrl5EnAar),
					Ctrl5CsaGain0V25) && ok;

	// Control_6 (buck regulator) is deliberately left untouched. On an EVM the buck may be supplying the
	// board's own logic rail, and reconfiguring or disabling it is an effective way to brown out the
	// hardware mid-bring-up. Change this only against a known schematic.

	configVerified = ok;
	return ok;
}

void DRV8316::PollFaults() noexcept
{
	lastIcStatus = ReadReg(RegIcStatus);
	if ((lastIcStatus & IcStatusFault) != 0)		// FAULT bit set — read detail registers
	{
		lastStatus1 = ReadReg(RegStatus1);
		lastStatus2 = ReadReg(RegStatus2);
	}
	else
	{
		lastStatus1 = 0;
		lastStatus2 = 0;
	}
}

void DRV8316::ClearFaults() noexcept
{
	// Read-modify-write so the PWM mode and slew rate configured by Init() survive. The hardware clears
	// CLR_FLT itself after the write, so there is nothing to clear afterwards.
	(void)UpdateReg(RegControl2, Ctrl2ClrFlt, Ctrl2ClrFlt);
	lastIcStatus = 0;
	lastStatus1  = 0;
	lastStatus2  = 0;
}

void DRV8316::AppendFaultDescription(const StringRef& reply) const noexcept
{
	// NPOR is informational rather than a fault - it reports that the device has powered up / been reset
	// since the flags were last cleared - so it is decoded even when the FAULT bit is clear. Reporting
	// "no faults" against a non-zero IC_Status is needlessly confusing.
	if ((lastIcStatus & IcStatusNpor) != 0)
	{
		reply.cat(" POR(reset since last clear)");
	}

	if (!HasFault())
	{
		if ((lastIcStatus & ~IcStatusNpor) == 0)
		{
			reply.cat(" no faults");
		}
		return;
	}

	// IC_Status summary bits
	if ((lastIcStatus & IcStatusOcp) != 0)    { reply.cat(" OCP"); }
	if ((lastIcStatus & IcStatusOt) != 0)     { reply.cat(" OT"); }
	if ((lastIcStatus & IcStatusOvp) != 0)    { reply.cat(" OVP"); }
	if ((lastIcStatus & IcStatusNpor) != 0)   { reply.cat(" POR"); }
	if ((lastIcStatus & IcStatusSpiFlt) != 0) { reply.cat(" SPI"); }
	if ((lastIcStatus & IcStatusBkFlt) != 0)  { reply.cat(" BUCK"); }

	// Status_1: which FET tripped over-current, plus thermal detail
	if ((lastStatus1 & Status1OcpHa) != 0) { reply.cat(" OCP:Ah"); }
	if ((lastStatus1 & Status1OcpLa) != 0) { reply.cat(" OCP:Al"); }
	if ((lastStatus1 & Status1OcpHb) != 0) { reply.cat(" OCP:Bh"); }
	if ((lastStatus1 & Status1OcpLb) != 0) { reply.cat(" OCP:Bl"); }
	if ((lastStatus1 & Status1OcpHc) != 0) { reply.cat(" OCP:Ch"); }
	if ((lastStatus1 & Status1OcpLc) != 0) { reply.cat(" OCP:Cl"); }
	if ((lastStatus1 & Status1Ots) != 0)   { reply.cat(" OT:shutdown"); }
	if ((lastStatus1 & Status1Otw) != 0)   { reply.cat(" OT:warning"); }

	// Status_2: SPI framing, charge pump, buck, one-time-programming
	if ((lastStatus2 & Status2SpiAddrFlt) != 0) { reply.cat(" SPI:addr"); }
	if ((lastStatus2 & Status2SpiSclkFlt) != 0) { reply.cat(" SPI:sclk"); }
	if ((lastStatus2 & Status2SpiParity) != 0)  { reply.cat(" SPI:parity"); }
	if ((lastStatus2 & Status2VcpUv) != 0)      { reply.cat(" VCP:undervolt"); }
	if ((lastStatus2 & Status2BuckUv) != 0)     { reply.cat(" BUCK:undervolt"); }
	if ((lastStatus2 & Status2BuckOcp) != 0)    { reply.cat(" BUCK:overcurrent"); }
	if ((lastStatus2 & Status2OtpErr) != 0)     { reply.cat(" OTP:error"); }
}
