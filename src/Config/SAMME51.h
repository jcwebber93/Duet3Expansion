/*
 * SAMME51.h
 * Generic ATSAME51G19 board with discrete stepper driver, PDEC encoder, and optional DC servo
 */

#ifndef SRC_CONFIG_SAMME51_H_
#define SRC_CONFIG_SAMME51_H_

#include <Hardware/PinDescription.h>
#include <SPI/SpiParameters.h>
#include <I2C/I2cParameters.h>
#include <UART/UartParameters.h>

#define BOARD_TYPE_NAME		"SAMME51"
#define BOOTLOADER_NAME		"SAMME51"

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		0
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				0

// Drivers
#define SUPPORT_DRIVERS			1
#define HAS_SMART_DRIVERS		0
#define HAS_STALL_DETECT		0
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	1
#define DEDICATED_STEP_TIMER	1
#define SUPPORT_INPUT_SHAPING	1
#define SUPPORT_CLOSED_LOOP		1
#define SUPPORT_DCSERVO			1
#define SUPPORT_BRAKE_PWM		0
#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_TMC2240_SPI		0

#define SUPPORT_MT6835			0
#define ACTIVE_HIGH_STEP		0
#define ACTIVE_HIGH_ENABLE		0	// active-low (standard for DRV8825, TMC standalone, etc.)
// FOC support has two supported hardware configurations on this board. Both use the same encoder,
// commutation, alignment and control-loop code; they differ only in how much the firmware knows about
// the gate driver sitting between the PWM outputs and the motor.
//
//   SUPPORT_DRV8316_SPI 1 - "smart driver". A TI DRV8316 on SERCOM4, configured and monitored over SPI:
//     PWM mode, slew rate, OCP threshold and CSA gain are set by firmware, faults are read back by
//     register and decoded in M122, and (once implemented) the driver's current-sense amplifier outputs
//     feed inline current sensing. PB08/PB09/PB11 are the SPI bus and PB10 is nSCS.
//
//   SUPPORT_DRV8316_SPI 0 - "dumb driver". A plain 3-PWM gate driver with no digital interface, e.g. the
//     SimpleFOCMini (DRV8313). Firmware drives three PWM phases and reads the encoder, and that is all
//     it can see; the only fault visibility is the driver's open-drain nFAULT wired to FocDriverFaultPin
//     below. PB08-PB11 revert to general-purpose GPIO. No current sensing is possible.
//
// Everything under SUPPORT_FOC is common to both, so switching this flag changes what the firmware can
// observe and configure, never how it commutates.
#define SUPPORT_FOC				1
#define SUPPORT_FOC_STEPPER		0
#define SUPPORT_DRV8316_SPI		1
#define SUPPORT_MT6835					0
#define SUPPORT_QUADRATURE_ENCODER		1
#define SUPPORT_COMPOSITE_ENCODER		0

// DMA channel assignments. NumDmaChannelsUsed must be at least the number of channels used; max 12 on
// the SAME5x. The DRV8316 pair only exists in the SPI-driver configuration (see SUPPORT_DRV8316_SPI
// below), so the count tracks it rather than reserving two channels that nothing can reach.
constexpr DmaChannel DmacChanAdc0Rx = 0;
constexpr DmaChannel DmacChanSspiTx = 1;
constexpr DmaChannel DmacChanSspiRx = 2;
constexpr DmaChannel DmacChanLedTx = 3;
#if SUPPORT_DRV8316_SPI
constexpr DmaChannel DmacChanDrv8316Tx = 4;
constexpr DmaChannel DmacChanDrv8316Rx = 5;
// Phase-current scanning. The SAME5x ADC has no self-sequencing mux; it has DMA sequencing, where one
// channel feeds INPUTCTRL values into DSEQDATA between conversions and a second collects the results.
// Hence a pair. Deliberately NOT DmacChanAdc0Rx: that belongs to CoreN2G's AnalogIn, and although
// nothing enables an ADC0 channel on this board any more (the MCU temperature sensors moved to ADC1),
// sharing it would make this depend on that staying true.
constexpr DmaChannel DmacChanFocIsenseSeq = 6;		// writes INPUTCTRL -> ADC0->DSEQDATA
constexpr DmaChannel DmacChanFocIsenseRes = 7;		// reads ADC0->RESULT -> sample buffer
constexpr unsigned int NumDmaChannelsUsed = 8;
#else
constexpr unsigned int NumDmaChannelsUsed = 4;
#endif


constexpr DmaPriority DmacPrioAdcRx = 2;
constexpr DmaPriority DmacPrioLed = 1;
constexpr DmaPriority DmacPrioSspiTx = 0;
constexpr DmaPriority DmacPrioSspiRx = 3;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const NvicPriority NvicPriorityStep = 3;
const NvicPriority NvicPriorityDmac = 3;
const NvicPriority NvicPriorityUart = 3;
const NvicPriority NvicPriorityI2C  = 3;
const NvicPriority NvicPriorityPins = 3;
const NvicPriority NvicPriorityCan  = 4;
const NvicPriority NvicPriorityAdc  = 5;

constexpr size_t NumDrivers = 1;
constexpr float MaxMotorCurrent = 1000.0;

#define ACTIVE_HIGH_DIR		1

// No physical stepper driver is fitted — SUPPORT_DRIVERS 1 is required for DC servo (Move class).

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_LIS3DH			0
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_LED_STRIPS		1
#define SUPPORT_DMA_NEOPIXEL	0	// SERCOM SPI idle-high bug on SAME51G19A makes DMA path unusable
#define USE_SERIAL_DEBUG		1
#define NUM_SERIAL_PORTS		1
#define NUM_I2C_CHANNELS		0
#define NUM_SHARED_SPI			1
#define NUM_ASYNC_PORTS			1

#define USE_MPU					0
#define USE_CACHE				1

constexpr unsigned int CANInstanceNumber = 0;
constexpr bool UseLaterCanPins = false;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 2;
constexpr float DefaultThermistorSeriesR = 2200.0;

// Thermistor on PA09 (adc0_9); PB09 and PA02 are general ADC GPIOs
constexpr Pin TempSensePins[NumThermistorInputs] = { PortAPin(9) };

// Diagnostic LEDs on SWDCLK/SWDIO (active low)
constexpr Pin LedPins[] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh = false;

// NeoPixel power enable on PB03 (active high)
constexpr Pin NeoPixelPWR = PortBPin(3);

#if SUPPORT_CLOSED_LOOP
// Shared SPI for closed-loop encoder comms (SERCOM1 on PA16/PA17/PA19)
constexpr uint8_t SspiSercomNumber = 1;
constexpr uint32_t SspiDataInPad = 3;
constexpr Pin SSPIMosiPin = PortAPin(16);
constexpr GpioPinFunction SSPIMosiPinPeriphMode = GpioPinFunction::C;
constexpr Pin SSPISclkPin = PortAPin(17);
constexpr GpioPinFunction SSPISclkPinPeriphMode = GpioPinFunction::C;
constexpr Pin SSPIMisoPin = PortAPin(19);
constexpr GpioPinFunction SSPIMisoPinPeriphMode = GpioPinFunction::C;
#endif
// Shared SPI definitions
constexpr SpiParameters SharedSpiParams =
{
	.sercomNumber = 1,
	.mosiPin = PortAPin(16),
	.misoPin = PortAPin(19),
	.sclkPin = PortAPin(17),
	.pinFunction = GpioPinFunction::C,
	.dataInPad = 3,
	.dataOutPad = 0,
	.dmaChanTx = DmacChanSspiTx,
	.dmaChanRx = DmacChanSspiRx,
	.dmaPrioTx = DmacPrioSspiTx,
	.dmaPrioRx = DmacPrioSspiRx,
};
// Encoder CS pin (used to deselect SPI encoders at startup, even when using PDEC)
constexpr Pin EncoderCsPin = PortAPin(18);

// Position decoder (PDEC): PA24=QDI0 (A), PA25=QDI1 (B), PB22=QDI2 (index)
constexpr Pin PositionDecoderPins[] = { PortAPin(24), PortAPin(25), PortBPin(22) };
constexpr GpioPinFunction PositionDecoderPinFunction = GpioPinFunction::G;

#if SUPPORT_FOC
// FOC three-phase PWM outputs — all on TCC0, Mux F, synchronized to one counter
// PA08 → TCC0/WO[0] (Mux F): phase W (motor phase sequence requires U on PA10, W on PA08)
// PA09 → TCC0/WO[1] (Mux F): phase V
// PA10 → TCC0/WO[2] (Mux F): phase U
constexpr Pin FocPhaseUPin = PortAPin(10);  // U on PA10
constexpr Pin FocPhaseVPin = PortAPin(9);   // V on PA09 unchanged
constexpr Pin FocPhaseWPin = PortAPin(8);   // W on PA08
constexpr GpioPinFunction FocPhaseUFn = GpioPinFunction::F;
constexpr GpioPinFunction FocPhaseVFn = GpioPinFunction::F;
constexpr GpioPinFunction FocPhaseWFn = GpioPinFunction::F;
constexpr PwmFrequency FocPwmFrequency = 20000;

// All three FOC phases share TCC0 on this board, so FocController drives the timer directly in
// centre-aligned (dual-slope) mode instead of going through AnalogOut::Write(). This is required for
// inline current sensing: phase current must be sampled in the middle of the window where all three
// low-side FETs are on, which only a centre-aligned carrier provides, and AnalogWriteTcc() in
// CoreN2G hard-codes single-slope NPWM for every PWM consumer on the board.
//
// TCC0 IS THEREFORE OWNED BY FocController on this board - do not route any other output to it.
// Boards whose phases sit on independent TCCs (EXP1HCL uses TCC0/TCC1/TCC2) simply leave this
// undefined and keep the edge-aligned AnalogOut path.
#define FOC_CENTRE_ALIGNED_PWM	1
constexpr unsigned int FocPwmTccNumber = 0;			// TCC0
constexpr unsigned int FocPhaseUTccChannel = 2;		// PA10 -> TCC0/WO[2]
constexpr unsigned int FocPhaseVTccChannel = 1;		// PA09 -> TCC0/WO[1]
constexpr unsigned int FocPhaseWTccChannel = 0;		// PA08 -> TCC0/WO[0]

// Bring-up aid, OFF by default. Set to 1 to pulse a GPIO from the TCC0 overflow interrupt, i.e. at the
// exact instant the current-sense ADC trigger fires.
//
// Scoping that pulse against any phase output answers the one question a scope trace of the phases alone
// cannot: the three phase pulses share a common centre (that is what centre-aligned means), but that
// centre sits at either counter TOP or counter BOTTOM, and the overflow event fires at BOTTOM. If the
// high pulses centre on TOP then BOTTOM is the middle of the window where all three low-side FETs
// conduct - which is where phase current must be sampled. If they centre on BOTTOM instead, the trigger
// lands mid-way through the all-HIGH window and the samples are useless.
//
// Best measured at standstill with zero torque, where SVPWM puts all three phases at 50% duty and the
// all-low and all-high windows are each ~25us. A ~1us marker in the middle of one of them is
// unambiguous, so interrupt latency is irrelevant at this resolution.
#define FOC_PWM_EVENT_DEBUG_PIN	0
#if FOC_PWM_EVENT_DEBUG_PIN
constexpr Pin FocPwmEventDebugPin = PortAPin(21);	// PA21 - free, no ADC or peripheral function
constexpr NvicPriority NvicPriorityFocPwmEventDebug = 3;	// same as Step, so it cannot pre-empt step timing
#endif
// SUPPORT_DCSERVO=1 is required to suppress stepper-driver pin requirements in Move.h.
// This board has no DC servo H-bridge; these are NoPin stubs so InitDcPwm/SetDcPwm compile.
constexpr Pin DcServoFwdPin = NoPin;
constexpr Pin DcServoRevPin = NoPin;
#elif SUPPORT_DCSERVO
// DC servo PWM — two independent TCCs for independent frequency control
// PA13 -> TCC1_WO3 (Mux G): servo forward / H-bridge IN A
// PA20 -> TCC0_WO0 (Mux G): servo reverse / H-bridge IN B
constexpr Pin DcServoFwdPin = PortAPin(20);		// io_fwd
constexpr Pin DcServoRevPin = PortAPin(13);		// io_rev
#endif

// SERCOM pin assignments used in pin table
constexpr auto Sercom0dPad0 = SercomIo::sercom0d + SercomIo::pad0;
constexpr auto Sercom0dPad1 = SercomIo::sercom0d + SercomIo::pad1;
constexpr auto Sercom5dPad0 = SercomIo::sercom5d + SercomIo::pad0;	// PB02 NeoPixel data out
#if SUPPORT_DRV8316_SPI
constexpr auto Sercom4dPad0 = SercomIo::sercom4d + SercomIo::pad0;	// PB08 DRV8316 MOSI
constexpr auto Sercom4dPad1 = SercomIo::sercom4d + SercomIo::pad1;	// PB09 DRV8316 SCK
constexpr auto Sercom4dPad3 = SercomIo::sercom4d + SercomIo::pad3;	// PB11 DRV8316 MISO
#endif

#if SUPPORT_DRV8316_SPI
// DRV8316 SPI on SERCOM4 (PB08=MOSI pad0, PB09=SCK pad1, PB11=MISO pad3, PB10=nSCS GPIO)
// All on Mux D. SERCOM4 is otherwise unused on SAME51G19A.
constexpr SpiParameters Drv8316SpiParams =
{
	.sercomNumber = 4,
	.mosiPin      = PortBPin(8),
	.misoPin      = PortBPin(11),
	.sclkPin      = PortBPin(9),
	.pinFunction  = GpioPinFunction::D,
	.dataInPad    = 3,
	.dataOutPad   = 0,
	.dmaChanTx    = DmacChanDrv8316Tx,
	.dmaChanRx    = DmacChanDrv8316Rx,
	.dmaPrioTx    = DmacPrioSspiTx,
	.dmaPrioRx    = DmacPrioSspiRx,
};
constexpr Pin Drv8316CsPin = PortBPin(10);
#endif

#if SUPPORT_FOC
// nFAULT from the 3-phase gate driver, open-drain active-low. Useful on both hardware configurations:
// on a DRV8313/SimpleFOCMini it is the ONLY fault visibility there is, and on a DRV8316 it is the only
// one that still works when the SPI link does not.
//
// PA12 rather than an ADC-capable pin: PA02/PA03/PA06/PA07 are the only free analog inputs and are
// needed for current sensing below. PA12 is also the only free non-ADC pin with an external interrupt
// line (EXINT 12), which is what an interrupt-driven fault path would need to get the DRV8316 register
// reads off the control loop.
constexpr Pin FocDriverFaultPin = PortAPin(12);		// PA12 - reserved in PinTable, EXINT 12
#endif

#if SUPPORT_DRV8316_SPI
// Inline current sensing from the DRV8316's current-sense amplifiers.
//
// The CSA outputs are bidirectional, centred on the DRV8316's own VREF/2 and swinging over 0..VREF. The
// TI EVM supplies VREF at 3.0 V by default, so zero current sits at 1.5 V and, at the 0.25 V/A gain
// configured in DRV8316::Init(), full scale is +-6.0 A.
//
// VREF is fed to the MCU's external analog reference (ANAREF/VREFA on PA03) rather than using an
// internal VDDANA-derived reference. That makes the measurement ratiometric: signal and reference move
// together, so the zero-current point stays at exactly mid-scale regardless of how VREF drifts. The
// alternative - internal reference, PA03 free as a third sense input - leaves an extra drift term for
// the offset calibration to chase for no benefit, since PA07 is available as the third channel anyway.
//
// All three channels are on ADC0 so a single TCC0 BOTTOM event (see FOC_CENTRE_ALIGNED_PWM) can trigger
// one DMA-sequenced conversion set. Sampling all three rather than deriving the third from
// Ia + Ib + Ic = 0 costs one conversion and buys that identity as a live sanity check.
constexpr Pin FocCurrentSenseAPin = PortAPin(2);		// ISENA -> ADC0/AIN0
constexpr Pin FocCurrentSenseBPin = PortAPin(6);		// ISENB -> ADC0/AIN6
constexpr Pin FocCurrentSenseCPin = PortAPin(7);		// ISENC -> ADC0/AIN7
// The DRV8316's own VREF sets the CSA mid-point and full-scale swing. 3.0 V is the TI EVM's default
// VREF select, so the CSA output is 1.5 V +-1.5 V.
constexpr float FocCurrentSenseVref = 3.0;			// volts, the DRV8316's reference - NOT the MCU's

// Which reference the MCU's ADC measures against. Two valid arrangements:
//
//   0 (default) - internal VDDANA reference, nothing to wire. The CSA's 0..3.0 V swing sits inside the
//       3.3 V range, so zero current lands near 1862 counts rather than mid-scale. VREF and VDDANA are
//       independent regulators, so their ratio can drift; the zero-offset calibration performed at every
//       alignment measures the actual zero point and absorbs it.
//
//   1 - ratiometric, VREF driven into the MCU's ANAREF/VREFA on PA03. Signal and reference then move
//       together, so zero current sits at exactly mid-scale by construction and stays there regardless
//       of drift. Slightly better, but the TI EVM does not bring VREF out on a header pin - its VREF
//       header pin is an INPUT for when an external MCU supplies the reference - so this needs a wire
//       from the EVM's VREF test point to PA03.
//
// Note PA03 is reserved in the PinTable either way, since it cannot be a general-purpose port while it
// is a candidate reference input.
#define FOC_CURRENT_SENSE_EXTERNAL_VREF	0
constexpr Pin FocCurrentSenseVrefPin = PortAPin(3);	// ANAREF/VREFA, used only when the above is 1

// Event system channel carrying TCC0 overflow to the ADC's start-conversion input. No EVSYS channel is
// otherwise used on this board. Must be 0..11 - only those channels have a GCLK, which the
// resynchronised path requires.
constexpr unsigned int FocCurrentSenseEventChannel = 0;

// ADC0 is taken over wholesale by FocCurrentSense, so the MCU temperature sensors move to ADC1, which is
// otherwise idle on this board. Enabling any channel on an ADC is what makes CoreN2G's AnalogIn service
// claim and configure it, so this is the whole of what "claiming ADC0" requires here.
//
// UNVERIFIED: whether ADC1's input mux is physically wired to the PTAT/CTAT sensors. The datasheet says
// only that the sensor is "routed to an ADC input channel", and the ASF headers define the mux values
// per peripheral type rather than per instance, so neither settles it. Test by checking that M122 still
// reports a plausible MCU temperature that tracks when the chip is warmed. If it does not, set
// HAS_CPU_TEMP_SENSOR to 0 for this board - the reading is a nicety on a development board, and ADC0 is
// not negotiable for current sensing since PA02/PA06/PA07 are ADC0-only.
#define MCU_TEMP_ADC_NUMBER		1
#endif


// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out			Exint	PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA00
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA01
#if SUPPORT_DRV8316_SPI
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	SercomIo::none,		SercomIo::none,		3,		nullptr			},	// PA02 FOC current sense A (ADC0/AIN0) — reserved
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA03 ANAREF/VREFA — external ADC reference, reserved
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	SercomIo::none,		SercomIo::none,		3,		"pa02"			},	// PA02
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	SercomIo::none,		SercomIo::none,		Nx,		"pa03"			},	// PA03
#endif
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,	SercomIo::none,		Sercom0dPad0,		4,		"pa04"			},	// PA04 UART TX (SERCOM0 Mux D pad 0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_5,	Sercom0dPad1,		SercomIo::none,		5,		"pa05"			},	// PA05 UART RX (SERCOM0 Mux D pad 1)
#if SUPPORT_DRV8316_SPI
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA06 FOC current sense B (ADC0/AIN6) — reserved
	{ TcOutput::tc1_1,	TccOutput::none,	AdcInput::adc0_7,	SercomIo::none,		SercomIo::none,		7,		nullptr			},	// PA07 FOC current sense C (ADC0/AIN7) — reserved
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	SercomIo::none,		SercomIo::none,		Nx,		"pa06"			},	// PA06
	{ TcOutput::tc1_1,	TccOutput::none,	AdcInput::adc0_7,	SercomIo::none,		SercomIo::none,		7,		"pa07"			},	// PA07
#endif
#if SUPPORT_FOC
	{ TcOutput::none,	TccOutput::tcc0_0F,	AdcInput::adc0_8,	SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA08 FOC phase W (TCC0_WO0, Mux F) — reserved
	{ TcOutput::none,	TccOutput::tcc0_1F,	AdcInput::adc0_9,	SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA09 FOC phase V (TCC0_WO1, Mux F) — reserved
	{ TcOutput::none,	TccOutput::tcc0_2F,	AdcInput::adc0_10,	SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA10 FOC phase U (TCC0_WO2, Mux F) — reserved
#else
	{ TcOutput::none,	TccOutput::tcc0_0F,	AdcInput::adc0_8,	SercomIo::none,		SercomIo::none,		Nx,		"pa08"			},	// PA08
	{ TcOutput::none,	TccOutput::tcc0_1F,	AdcInput::adc0_9,	SercomIo::none,		SercomIo::none,		Nx,		"pa09"			},	// PA09
	{ TcOutput::none,	TccOutput::tcc0_2F,	AdcInput::adc0_10,	SercomIo::none,		SercomIo::none,		Nx,		"pa10"			},	// PA10
#endif
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_11,	SercomIo::none,		SercomIo::none,		Nx,		"pa11"			},	// PA11, could be used for EN
#if SUPPORT_FOC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,		nullptr			},	// PA12 FocDriverFaultPin (gate driver nFAULT) — reserved
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,		"pa12"			},	// PA12
#endif
#if SUPPORT_FOC
	{ TcOutput::none,	TccOutput::tcc1_3G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		13,		nullptr			},	// PA13 FOC phase U (TCC1_WO3, Mux G) — reserved
#elif SUPPORT_DCSERVO
	{ TcOutput::none,	TccOutput::tcc1_3G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		13,		"pa13"			},	// PA13 DC servo fwd (TCC1_WO3, Mux G)
#else
	{ TcOutput::none,	TccOutput::tcc1_3G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		13,		"pa13"			},	// PA13
#endif
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA14 XIN (crystal)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA15 XOUT (crystal)
#if SUPPORT_CLOSED_LOOP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,		nullptr			},	// PA16 SPI MOSI (SERCOM1, reserved for encoder)
	{ TcOutput::tc2_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		1,		nullptr			},	// PA17 SPI SCK  (SERCOM1, reserved for encoder)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,		nullptr			},	// PA18 encoder CS (reserved)
	{ TcOutput::tc3_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA19 SPI MISO (SERCOM1, reserved for encoder)
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		0,		"pa16"			},	// PA16
	{ TcOutput::tc2_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		1,		"pa17"			},	// PA17
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		2,		"pa18"			},	// PA18
	{ TcOutput::tc3_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa19"			},	// PA19
#endif
#if SUPPORT_FOC
	{ TcOutput::none,	TccOutput::tcc0_0G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA20 FOC phase V (TCC0_WO0, Mux G) — reserved
#elif SUPPORT_DCSERVO
	{ TcOutput::none,	TccOutput::tcc0_0G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa20"			},	// PA20 DC servo rev (TCC0_WO0, Mux G)
#else
	{ TcOutput::none,	TccOutput::tcc0_0G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa20"			},	// PA20
#endif
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa21"			},	// PA21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA22 CAN0 TX
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA23 CAN0 RX
	{ TcOutput::none,	TccOutput::tcc2_2F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		8,		"pa24"			},	// PA24 PDEC QDI0 (encoder A)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,		"pa25"			},	// PA25 PDEC QDI1 (encoder B)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa27"			},	// PA27
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA30 SWDCLK / LED0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA31 SWDIO / LED1

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB00 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB01 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		Sercom5dPad0,		Nx,		"led"			},	// PB02 NeoPixel data
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB03 NeoPixel power enable
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB07 not on chip
#if SUPPORT_DRV8316_SPI
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	SercomIo::none,		Sercom4dPad0,		Nx,		nullptr			},	// PB08 DRV8316 MOSI (SERCOM4 Mux D pad0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	SercomIo::none,		Sercom4dPad1,		Nx,		nullptr			},	// PB09 DRV8316 SCK  (SERCOM4 Mux D pad1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,		nullptr			},	// PB10 DRV8316 nSCS (GPIO, active low)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		Sercom4dPad3,		SercomIo::none,		11,		nullptr			},	// PB11 DRV8316 MISO (SERCOM4 Mux D pad3)
#else
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	SercomIo::none,		SercomIo::none,		Nx,		"pb08"			},	// PB08
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	SercomIo::none,		SercomIo::none,		Nx,		"pb09"			},	// PB09 (TC4 does not exist on SAME51G19A)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,		"pb10"			},	// PB10
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,		"pb11"			},	// PB11
#endif
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB12 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB13 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB14 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB15 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB16 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB17 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB18 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB19 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB20 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB21 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,		"pb22"			},	// PB22 PDEC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pb23"			},	// PB23
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32 + 24;				// 32 pins on port A (some missing), 24 on port B
static_assert(NumPins == NumRealPins);

// Step timer: TC0 (32-bit, consumes TC0+TC1 — neither tc0_* nor tc1_* in pin table)
TcCount32 * const StepTc = &(TC0->COUNT32);
constexpr IRQn StepTcIRQn = TC0_IRQn;
constexpr unsigned int StepTcNumber = 0;
#define STEP_TC_HANDLER		TC0_Handler

// UART on SERCOM0 (PA04 = TX pad 0, PA05 = RX pad 1, Mux D)
constexpr IRQn Serial0_IRQn = SERCOM0_0_IRQn;

constexpr UartParameters Serial0Params =
{
	.sercomNumber = 0,
	.rxPin = PortAPin(5),
	.txPin = PortAPin(4),
	.pinFunction = GpioPinFunction::D,
	.dataInPad = 1,
	.dataOutPad = 0,
	.numRxSlots = 32,
	.numTxSlots = 512
};

#endif /* SRC_CONFIG_SAMME51_H_ */
