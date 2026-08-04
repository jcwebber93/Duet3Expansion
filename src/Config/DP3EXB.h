/*
 * DP3EXB.h
 * Custom ATSAME51G19 expansion board with generic I/O and DC servo/encoder support
 */

#ifndef SRC_CONFIG_DP3EXB_H_
#define SRC_CONFIG_DP3EXB_H_

#include <Hardware/PinDescription.h>
#include <SPI/SpiParameters.h>
#include <I2C/I2cParameters.h>

#define BOARD_TYPE_NAME		"DP3EXB"
#define BOOTLOADER_NAME		"DP3EXB"

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

#define SUPPORT_FOC				0
#define SUPPORT_FOC_STEPPER		0
#define SUPPORT_MT6835					0
#define SUPPORT_QUADRATURE_ENCODER		1
#define SUPPORT_COMPOSITE_ENCODER		0

// DMA channel assignments
constexpr DmaChannel DmacChanAdc0Rx = 0;
constexpr DmaChannel DmacChanSspiTx = 1;
constexpr DmaChannel DmacChanSspiRx = 2;

constexpr unsigned int NumDmaChannelsUsed = 3;

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
#define NUM_SERIAL_PORTS		0

#define NUM_I2C_CHANNELS		0
#define NUM_SHARED_SPI			1
#define NUM_ASYNC_PORTS			0

#define USE_MPU					0
#define USE_CACHE				1

constexpr unsigned int CANInstanceNumber = 0;
constexpr bool UseLaterCanPins = false;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 1;
constexpr float DefaultThermistorSeriesR = 2200.0;

// Thermistor on PA09 (adc0_9); PB09 is general ADC GPIO; PB08 is a switch input
constexpr Pin TempSensePins[NumThermistorInputs] = { PortAPin(9) };

// Diagnostic LEDs on SWDCLK/SWDIO (active low, same as EXP1HCL)
constexpr Pin LedPins[] = { PortAPin(30), PortAPin(31) };
constexpr bool LedActiveHigh = false;

// Shared SPI for closed-loop encoder comms (SERCOM1 on PA16/PA17/PA19, same as EXP1HCL)
// PA16 = MOSI (pad 0), PA17 = SCK (pad 1), PA19 = MISO (pad 3)
constexpr uint8_t SspiSercomNumber = 1;
constexpr uint32_t SspiDataInPad = 3;
constexpr Pin SSPIMosiPin = PortAPin(16);
constexpr GpioPinFunction SSPIMosiPinPeriphMode = GpioPinFunction::C;
constexpr Pin SSPISclkPin = PortAPin(17);
constexpr GpioPinFunction SSPISclkPinPeriphMode = GpioPinFunction::C;
constexpr Pin SSPIMisoPin = PortAPin(19);
constexpr GpioPinFunction SSPIMisoPinPeriphMode = GpioPinFunction::C;

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

// Position decoder: PA24 = encoder A (PDEC QDI0), PA25 = encoder B (PDEC QDI1)
constexpr Pin PositionDecoderPins[] = { PortAPin(24), PortAPin(25) };
constexpr GpioPinFunction PositionDecoderPinFunction = GpioPinFunction::G;

#if SUPPORT_DCSERVO
// DC servo PWM — two independent TCCs for independent frequency control
// PA13 -> TCC1_WO3 (Mux G, tcc1_3G): servo forward / H-bridge IN A
// PA20 -> TCC0_WO0 (Mux G, tcc0_0G): servo reverse / H-bridge IN B
//constexpr Pin DcServoFwdPin = PortAPin(13);		// io_fwd
//constexpr Pin DcServoRevPin = PortAPin(20);		// io_rev
constexpr Pin DcServoFwdPin = PortAPin(20);		// io_fwd
constexpr Pin DcServoRevPin = PortAPin(13);		// io_rev
#endif

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out			Exint	PinName
	// Port A
	{ TcOutput::tc2_0,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa00"			},	// PA00 5V out (TC2 ch0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa01"			},	// PA01 5V out
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA02 disconnected
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA03 disconnected
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		4,		"pa04"			},	// PA04 5V in switch
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		5,		"pa05"			},	// PA05 5V in switch
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		6,		"pa06"			},	// PA06 5V in switch
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,		"pa07"			},	// PA07 5V in switch
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa08"			},	// PA08 5V out
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_9,	SercomIo::none,		SercomIo::none,		Nx,		"temp0"			},	// PA09 thermistor (ADC only)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa10"			},	// PA10 5V out
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa11"			},	// PA11 3.3V GPIO (TCC0 consumed by io_rev, TCC1 by io_fwd; TC1 blocked)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,		"pa12"			},	// PA12 5V in
	{ TcOutput::none,	TccOutput::tcc1_3G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"io_fwd"		},	// PA13 DC servo fwd (TCC1_WO3, Mux G)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA14 XIN (crystal)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA15 XOUT (crystal)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA16 SPI MOSI (SERCOM1, reserved for encoder) WRONG
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA17 SPI SCK  (SERCOM1, reserved for encoder) WRONG
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA18 encoder CS (reserved) WRONG
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA19 SPI MISO (SERCOM1, reserved for encoder) WRONG
	{ TcOutput::none,	TccOutput::tcc0_0G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"io_rev"		},	// PA20 DC servo rev (TCC0_WO0, Mux G)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa21"			},	// PA21 5V but exint conflict
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA22 CAN0 TX
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA23 CAN0 RX
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		8,		"pa24"			},	// PA24 PDEC QDI0 (encoder A, Mux G)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		9,		"pa25"			},	// PA25 PDEC QDI1 (encoder B, Mux G)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa27"			},	// PA27 5V out
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA30 SWDCLK / LED0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA31 SWDIO / LED1

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB00 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB01 not on chip
	{ TcOutput::none,	TccOutput::tcc2_2F,	AdcInput::adc0_14,	SercomIo::none,		SercomIo::none,		2,		"pb02"			},	// PB02 3.3V GPIO (TCC2 ch2 Mux F)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB03 disconnected
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB04 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB05 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB06 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB07 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB08 5V in switch but exint conflict
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_3,	SercomIo::none,		SercomIo::none,		Nx,		"pb09"			},	// PB09 3.3V GPIO (general ADC)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,		"pb10"			},	// PB10 3.3V GPIO (TC5 ch0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,		"pb11"			},	// PB11 5V in
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
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB22 disconnected
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pb23"			},	// PB23 5V out GPIO (tc7_1 consumed by PA21)
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32 + 24;				// 32 pins on port A (some missing), 24 on port B
static_assert(NumPins == NumRealPins);

// Step timer: TC0 (same as EXP1HCL — leaves TCC0/TCC1 free for DC servo PWM)
TcCount32 * const StepTc = &(TC0->COUNT32);
constexpr IRQn StepTcIRQn = TC0_IRQn;
constexpr unsigned int StepTcNumber = 0;
#define STEP_TC_HANDLER		TC0_Handler

#define NUM_SERIAL_PORTS	0



#endif /* SRC_CONFIG_DP3EXB_H_ */
