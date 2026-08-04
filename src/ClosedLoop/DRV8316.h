/*
 * DRV8316.h — TI DRV8316 gate driver SPI interface
 *
 * Handles register unlock, 3PWM mode init, and fault status polling.
 * Current sense and gate-drive configuration are out of scope for Phase 2.
 */

#ifndef SRC_CLOSEDLOOP_DRV8316_H_
#define SRC_CLOSEDLOOP_DRV8316_H_

#include <RepRapFirmware.h>
#include <SPI/SharedSpiDevice.h>
#include <SPI/SharedSpiClient.h>

class DRV8316
{
public:
	DRV8316(SharedSpiDevice& spi, Pin csPin) noexcept;

	// Unlock registers and configure 3PWM mode. Returns false if SPI comms appear dead.
	bool Init() noexcept;

	// Read IC_Status (and Status_1/2 if FAULT is set). Call at ~10 ms intervals.
	void PollFaults() noexcept;

	// Clear latched fault flags (CLR_FLT bit in Control_2).
	void ClearFaults() noexcept;

	// Fault accessors
	bool IsPresent() const noexcept { return spiPresent; }
	bool HasFault()  const noexcept { return lastIcStatus & 0x01u; }
	bool HasOcp()    const noexcept { return lastIcStatus & 0x10u; }	// over-current
	bool HasOt()     const noexcept { return lastIcStatus & 0x02u; }	// over-temperature shutdown or warning
	bool HasOtw()    const noexcept { return lastStatus1  & 0x80u; }	// over-temperature warning

	uint8_t GetIcStatus() const noexcept { return lastIcStatus; }
	uint8_t GetStatus1()  const noexcept { return lastStatus1;  }
	uint8_t GetStatus2()  const noexcept { return lastStatus2;  }

private:
	// Register addresses
	static constexpr uint8_t RegIcStatus  = 0x00;
	static constexpr uint8_t RegStatus1   = 0x01;
	static constexpr uint8_t RegStatus2   = 0x02;
	static constexpr uint8_t RegControl1  = 0x03;	// REG_LOCK
	static constexpr uint8_t RegControl2  = 0x04;	// CLR_FLT, PWM_MODE, SLEW
	static constexpr uint8_t RegControl4  = 0x06;	// DRV_OFF

	// Control_1 REG_LOCK values
	static constexpr uint8_t RegLockUnlocked = 0b011;
	static constexpr uint8_t RegLockLocked   = 0b110;

	// Control_2 bit definitions
	static constexpr uint8_t Ctrl2ClrFlt    = 0x01u;
	static constexpr uint8_t Ctrl2Pwm3Mode  = 0b10u << 1;	// PWM_MODE[1:0] = 0b10 → 3PWM

	uint8_t ReadReg(uint8_t addr) noexcept;
	void    WriteReg(uint8_t addr, uint8_t val) noexcept;

	// Build a 16-bit SPI frame with even parity.
	static uint16_t BuildFrame(bool read, uint8_t addr, uint8_t data) noexcept;

	SharedSpiClient spiClient;
	bool    spiPresent   = false;
	uint8_t lastIcStatus = 0;
	uint8_t lastStatus1  = 0;
	uint8_t lastStatus2  = 0;
};

#endif /* SRC_CLOSEDLOOP_DRV8316_H_ */
