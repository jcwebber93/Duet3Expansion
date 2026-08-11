/*
 * DRV8316.h — TI DRV8316 gate driver SPI interface
 *
 * Handles register unlock, driver configuration, write verification and fault status decoding.
 *
 * Register map, field layouts and enum encodings were cross-checked against the SimpleFOC
 * Arduino-FOC-drivers implementation in Arduino-FOC-drivers/src/drivers/drv8316/ (drv8316_registers.h
 * for addresses and bitfields, drv8316.h for the enum values). That is a widely-used community
 * implementation rather than TI's document, so treat the datasheet as final authority if the two ever
 * disagree — but every value here matched it exactly.
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

	// Unlock registers, apply the configuration below, and verify it read back correctly.
	// Returns false if SPI comms appear dead or the configuration did not stick.
	bool Init() noexcept;

	// Refresh IC_Status (and Status_1/2 if FAULT is set). Not safe to call from the control loop —
	// each register access is a blocking ~16us SPI transaction.
	void PollFaults() noexcept;

	// Clear latched fault flags (CLR_FLT bit in Control_2).
	void ClearFaults() noexcept;

	// Status accessors. Bit positions per IC_Status / Status_1 in drv8316_registers.h.
	bool IsPresent() const noexcept { return spiPresent; }
	bool IsConfigVerified() const noexcept { return configVerified; }
	bool HasFault() const noexcept { return (lastIcStatus & IcStatusFault) != 0; }
	bool HasOcp()   const noexcept { return (lastIcStatus & IcStatusOcp) != 0; }		// over-current
	bool HasOt()    const noexcept { return (lastIcStatus & IcStatusOt) != 0; }		// over-temperature shutdown
	bool HasOtw()   const noexcept { return (lastStatus1 & Status1Otw) != 0; }		// over-temperature warning

	uint8_t GetIcStatus() const noexcept { return lastIcStatus; }
	uint8_t GetStatus1()  const noexcept { return lastStatus1;  }
	uint8_t GetStatus2()  const noexcept { return lastStatus2;  }

	// Append a human-readable decode of the latched status bits, for M122.
	void AppendFaultDescription(const StringRef& reply) const noexcept;

	// Current-sense amplifier gain in volts per amp, matching the CSA_GAIN value configured by Init().
	// Needed by the current-sense path to convert ADC volts to amps:
	//     amps = (Vsense - Vref/2) / CsaGainVoltsPerAmp
	// The zero-current point is Vref/2, where Vref is the DRV8316's own reference (3.0 V by default on
	// the TI EVM), NOT the MCU's ADC reference. At 0.25 V/A that gives a +-6.0 A measurement range.
	static constexpr float CsaGainVoltsPerAmp = 0.25;

private:
	// ---- Register addresses ----
	static constexpr uint8_t RegIcStatus  = 0x00;
	static constexpr uint8_t RegStatus1   = 0x01;
	static constexpr uint8_t RegStatus2   = 0x02;
	static constexpr uint8_t RegControl1  = 0x03;	// REG_LOCK
	static constexpr uint8_t RegControl2  = 0x04;	// CLR_FLT, PWM_MODE, SLEW, SDO_MODE
	static constexpr uint8_t RegControl3  = 0x05;	// OTW_REP, SPI_FLT_REP, OVP_EN, OVP_SEL, PWM_100_DUTY_SEL
	static constexpr uint8_t RegControl4  = 0x06;	// OCP_MODE, OCP_LVL, OCP_RETRY, OCP_DEG, OCP_CBC, DRV_OFF
	static constexpr uint8_t RegControl5  = 0x07;	// CSA_GAIN, EN_ASR, EN_AAR, ILIM_RECIR
	static constexpr uint8_t RegControl6  = 0x08;	// BUCK_DIS, BUCK_SEL, BUCK_CL, BUCK_PS_DIS
	static constexpr uint8_t RegControl10 = 0x0C;	// DLY_TARGET, DLYCMP_EN

	// ---- IC_Status bits ----
	static constexpr uint8_t IcStatusFault  = 1u << 0;
	static constexpr uint8_t IcStatusOt     = 1u << 1;
	static constexpr uint8_t IcStatusOvp    = 1u << 2;
	static constexpr uint8_t IcStatusNpor   = 1u << 3;
	static constexpr uint8_t IcStatusOcp    = 1u << 4;
	static constexpr uint8_t IcStatusSpiFlt = 1u << 5;
	static constexpr uint8_t IcStatusBkFlt  = 1u << 6;

	// ---- Status_1 bits (per-FET over-current, plus thermal) ----
	static constexpr uint8_t Status1OcpLa = 1u << 0;
	static constexpr uint8_t Status1OcpHa = 1u << 1;
	static constexpr uint8_t Status1OcpLb = 1u << 2;
	static constexpr uint8_t Status1OcpHb = 1u << 3;
	static constexpr uint8_t Status1OcpLc = 1u << 4;
	static constexpr uint8_t Status1OcpHc = 1u << 5;
	static constexpr uint8_t Status1Ots   = 1u << 6;
	static constexpr uint8_t Status1Otw   = 1u << 7;

	// ---- Status_2 bits (SPI, charge pump, buck, OTP) ----
	static constexpr uint8_t Status2SpiAddrFlt = 1u << 0;
	static constexpr uint8_t Status2SpiSclkFlt = 1u << 1;
	static constexpr uint8_t Status2SpiParity  = 1u << 2;
	static constexpr uint8_t Status2VcpUv      = 1u << 3;
	static constexpr uint8_t Status2BuckUv     = 1u << 4;
	static constexpr uint8_t Status2BuckOcp    = 1u << 5;
	static constexpr uint8_t Status2OtpErr     = 1u << 6;

	// ---- Control_1: REG_LOCK[2:0] ----
	static constexpr uint8_t Ctrl1RegLockMask     = 0x07;
	static constexpr uint8_t Ctrl1RegLockUnlocked = 0b011;
	static constexpr uint8_t Ctrl1RegLockLocked   = 0b110;

	// ---- Control_2: CLR_FLT[0], PWM_MODE[2:1], SLEW[4:3], SDO_MODE[5] ----
	static constexpr uint8_t Ctrl2ClrFlt       = 1u << 0;
	static constexpr uint8_t Ctrl2PwmModeShift = 1;
	static constexpr uint8_t Ctrl2PwmModeMask  = 0b11u << Ctrl2PwmModeShift;
	static constexpr uint8_t Ctrl2PwmMode3Pwm  = 0b10u << Ctrl2PwmModeShift;	// PWM3_Mode
	static constexpr uint8_t Ctrl2SlewShift    = 3;
	static constexpr uint8_t Ctrl2SlewMask     = 0b11u << Ctrl2SlewShift;
	static constexpr uint8_t Ctrl2Slew50Vus    = 0b01u << Ctrl2SlewShift;

	// ---- Control_4: OCP_MODE[1:0], OCP_LVL[2], OCP_RETRY[3], OCP_DEG[5:4], OCP_CBC[6], DRV_OFF[7] ----
	static constexpr uint8_t Ctrl4OcpModeShift   = 0;
	static constexpr uint8_t Ctrl4OcpModeMask    = 0b11u << Ctrl4OcpModeShift;
	static constexpr uint8_t Ctrl4OcpModeLatched = 0b00u << Ctrl4OcpModeShift;
	static constexpr uint8_t Ctrl4OcpLvl         = 1u << 2;			// 0 = 16A, 1 = 24A
	static constexpr uint8_t Ctrl4OcpDegShift    = 4;
	static constexpr uint8_t Ctrl4OcpDegMask     = 0b11u << Ctrl4OcpDegShift;
	static constexpr uint8_t Ctrl4OcpDeg1us1     = 0b10u << Ctrl4OcpDegShift;
	static constexpr uint8_t Ctrl4OcpCbc         = 1u << 6;
	static constexpr uint8_t Ctrl4DrvOff         = 1u << 7;

	// ---- Control_5: CSA_GAIN[1:0], EN_ASR[2], EN_AAR[3], ILIM_RECIR[6] ----
	static constexpr uint8_t Ctrl5CsaGainShift = 0;
	static constexpr uint8_t Ctrl5CsaGainMask  = 0b11u << Ctrl5CsaGainShift;
	static constexpr uint8_t Ctrl5CsaGain0V25  = 0b10u << Ctrl5CsaGainShift;	// 0.25 V/A, see CsaGainVoltsPerAmp
	static constexpr uint8_t Ctrl5EnAsr        = 1u << 2;
	static constexpr uint8_t Ctrl5EnAar        = 1u << 3;

	uint8_t ReadReg(uint8_t addr) noexcept;
	void    WriteReg(uint8_t addr, uint8_t val) noexcept;

	// Read-modify-write: only the bits in 'mask' are replaced by 'val'. Returns true if the register
	// read back with the intended value. Every DRV8316 control register holds unrelated settings, so a
	// blind write silently resets whatever else lives there.
	bool UpdateReg(uint8_t addr, uint8_t mask, uint8_t val) noexcept;

	// Perform one 16-bit transaction. The device returns IC_Status in the first byte of EVERY
	// transaction (reads and writes alike) and the register content in the second, so this captures the
	// status for free and hands back the data byte.
	uint8_t Transfer(uint16_t frame) noexcept;

	// Build a 16-bit SPI frame with even parity: [15]=R/W, [14:9]=addr, [8]=parity, [7:0]=data.
	static uint16_t BuildFrame(bool read, uint8_t addr, uint8_t data) noexcept;

	SharedSpiClient spiClient;
	bool    spiPresent    = false;
	bool    configVerified = false;
	uint8_t lastIcStatus  = 0;
	uint8_t lastStatus1   = 0;
	uint8_t lastStatus2   = 0;
};

#endif /* SRC_CLOSEDLOOP_DRV8316_H_ */
