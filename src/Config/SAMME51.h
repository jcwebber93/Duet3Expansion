/*
 * SAMME51.h
 * Generic ATSAME51G19 board with discrete stepper driver, PDEC encoder, and optional DC servo
 */

#ifndef SRC_CONFIG_SAMME51_H_
#define SRC_CONFIG_SAMME51_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME		"SAMME51"
#define BOOTLOADER_NAME		"SAME5x"

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
#define SUPPORT_MT6835			0
#define ACTIVE_HIGH_STEP		1
#define ACTIVE_HIGH_DIR			1
#define ACTIVE_HIGH_ENABLE		0	// active-low (standard for DRV8825, TMC standalone, etc.)

constexpr size_t NumDrivers = 1;

PortGroup * const StepPio = &(PORT->Group[0]);
constexpr Pin StepPins[NumDrivers]      = { PortAPin(9) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(10) };
constexpr Pin EnablePins[NumDrivers]    = { PortAPin(11) };

#define SUPPORT_THERMISTORS		1
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_LIS3DH			0
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_LED_STRIPS		1
#define SUPPORT_DMA_NEOPIXEL	1
#define USE_SERIAL_DEBUG		1
#define NUM_SERIAL_PORTS		1

#define USE_MPU					0
#define USE_CACHE				1

// CAN0 on standard pins PA22/PA23
constexpr bool UseAlternateCanPins = false;

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

// Encoder CS pin (used to deselect SPI encoders at startup, even when using PDEC)
constexpr Pin EncoderCsPin = PortAPin(18);

// Position decoder
constexpr Pin PositionDecoderPins[] = { PortAPin(24), PortAPin(25), PortBPin(22) };
constexpr GpioPinFunction PositionDecoderPinFunction = GpioPinFunction::G;

#if SUPPORT_DCSERVO
// DC servo PWM — two independent TCCs for independent frequency control
// PA13 -> TCC1_WO3 (Mux G, tcc1_3G): servo forward / H-bridge IN A
// PA20 -> TCC0_WO0 (Mux G, tcc0_0G): servo reverse / H-bridge IN B
// TCC0 and TCC1 are fully consumed by these two pins; no other TCC0/TCC1 in pin table.
constexpr Pin DcServoFwdPin = PortAPin(13);		// io_fwd
constexpr Pin DcServoRevPin = PortAPin(20);		// io_rev
#endif

// SERCOM pin assignments used in pin table
constexpr auto Sercom0dPad0 = SercomIo::sercom0d + SercomIo::pad0;
constexpr auto Sercom0dPad1 = SercomIo::sercom0d + SercomIo::pad1;
constexpr auto Sercom5dPad0 = SercomIo::sercom5d + SercomIo::pad0;	// PB02 NeoPixel data out


// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out			Exint	PinName
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA00
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA01
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,	SercomIo::none,		SercomIo::none,		3,		"pa02"			},	// PA02
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_1,	SercomIo::none,		SercomIo::none,		Nx,		"pa03"			},	// PA03
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,	SercomIo::none,		Sercom0dPad0,		4,		"pa04"			},	// PA04 UART TX (SERCOM0 Mux D pad 0)
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_5,	Sercom0dPad1,		SercomIo::none,		5,		"pa05"			},	// PA05 UART RX (SERCOM0 Mux D pad 1)
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa06"			},	// PA06
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		7,		"pa07"			},	// PA07
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_8,	SercomIo::none,		SercomIo::none,		Nx,		"pa08"			},	// PA08
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_9,	SercomIo::none,		SercomIo::none,		Nx,		"pa09"			},	// PA09, could be used for STEP
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_10,	SercomIo::none,		SercomIo::none,		Nx,		"pa10"			},	// PA10, could be used for DIR
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_11,	SercomIo::none,		SercomIo::none,		Nx,		"pa11"			},	// PA11, could be used for EN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,		"pa12"			},	// PA12
#if SUPPORT_DCSERVO	
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
#if SUPPORT_DCSERVO		
	{ TcOutput::none,	TccOutput::tcc0_0G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa20"			},	// PA20 DC servo rev (TCC0_WO0, Mux G)
#else
	{ TcOutput::none,	TccOutput::tcc0_0G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa20"			},	// PA20
#endif	
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa21"			},	// PA21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA22 CAN0 TX
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA23 CAN0 RX
	{ TcOutput::none,	TccOutput::tcc2_2f,	AdcInput::none,		SercomIo::none,		SercomIo::none,		8,		"pa24"			},	// PA24 PDEC QDI0 (encoder A)
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
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_2,	SercomIo::none,		SercomIo::none,		Nx,		"pb08"			},	// PB08
	{ TcOutput::tc4_1,	TccOutput::none,	AdcInput::adc0_3,	SercomIo::none,		SercomIo::none,		Nx,		"pb09"			},	// PB09
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		10,		"pb10"			},	// PB10
	{ TcOutput::tc5_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		11,		"pb11"			},	// PB11
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

// UART on SERCOM0 (PA04 = TX, PA05 = RX)
constexpr IRQn Serial0_IRQn = SERCOM0_0_IRQn;

// DMA channel assignments
constexpr DmaChannel DmacChanTmcTx = 0;
constexpr DmaChannel DmacChanTmcRx = 1;
constexpr DmaChannel DmacChanAdc0Rx = 2;
constexpr DmaChannel DmacChanLedTx = 3;
constexpr unsigned int NumDmaChannelsUsed = 4;

constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 3;
constexpr DmaPriority DmacPrioAdcRx = 2;
constexpr DmaPriority DmacPrioLed = 1;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const NvicPriority NvicPriorityStep = 3;
const NvicPriority NvicPriorityDmac = 3;
const NvicPriority NvicPriorityUart = 3;
const NvicPriority NvicPriorityI2C  = 3;
const NvicPriority NvicPriorityPins = 3;
const NvicPriority NvicPriorityCan  = 4;
const NvicPriority NvicPriorityAdc  = 5;

#endif /* SRC_CONFIG_SAMME51_H_ */
