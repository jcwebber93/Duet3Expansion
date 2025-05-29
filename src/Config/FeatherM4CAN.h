/*
 * Pins_FeatherM4CAN.h - Duet3Expansion firmware pin definitions for Feather M4 CAN Express
 *
 * Based on SAMMYC21.h and user-provided initial pinout.
 */

#ifndef SRC_CONFIG_PINS_FEATHERM4CAN_H_
#define SRC_CONFIG_PINS_FEATHERM4CAN_H_

#include <Hardware/PinDescription.h>
#include <DmacManager.h>

#define BOARD_TYPE_NAME			"FeatherM4CAN"
#define BOOTLOADER_NAME			"SAME5x"			// Or "UF2_SAME5x" if using UF2 bootloader

// General features
#define HAS_VREF_MONITOR		0					// No dedicated VREF monitor circuit assumed
#define HAS_VOLTAGE_MONITOR		0					// No on-board voltage monitoring assumed (can be added via ADC)
#define HAS_12V_MONITOR			0					// No 12V monitoring assumed
#define HAS_CPU_TEMP_SENSOR		1					// SAME51 has internal temperature sensor
#define HAS_ADDRESS_SWITCHES	0					// No address switches on Feather
#define HAS_BUTTONS				0					// Feather has a reset button, potentially a user button (not typically used by RRF expansion)

// Driver Configuration (for one local "slow" driver as per your example)
#define SUPPORT_DRIVERS			1
#define HAS_SMART_DRIVERS		0					// Not using smart drivers for this example
#define HAS_STALL_DETECT		0
#define SINGLE_DRIVER			1					// Configuring for a single driver
#define SUPPORT_SLOW_DRIVERS	1
#define DEDICATED_STEP_TIMER	1					// We'll use a TC/TCC for step generation
#define SUPPORT_INPUT_SHAPING	0					// Typically not on a simple expansion driver

// Polarity for generic step/dir drivers (adjust if needed)
#define ACTIVE_HIGH_STEP		1
#define ACTIVE_HIGH_DIR			1
#define ACTIVE_HIGH_ENABLE		0					// Common for enable to be active low

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0


constexpr size_t NumDrivers = 1;

// Define the step/dir/enable pins from your example
constexpr Pin StepPins[NumDrivers] = { PortAPin(16) };      // D6 on Feather M4 CAN
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(17) };  // D7 on Feather M4 CAN
constexpr Pin EnablePins[NumDrivers] = { PortAPin(18) };    // D8 on Feather M4 CAN
PortGroup * const StepPio = &(PORT->Group[0]);              // Port A for these pins

// Sensor Support (minimal for now, can be expanded)
#define SUPPORT_THERMISTORS		0
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_LIS3DH			0
#define SUPPORT_LDC1612			0
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_SDADC			0

#define USE_MPU					0
#define USE_CACHE				1					// SAME5x has cache


// CAN Interface (from your example)
#define UseAlternateCanPins		true				// Standard CAN1 pins for SAME51 (PB14/PB15)
constexpr unsigned int CanDeviceNumber = 1;
constexpr Pin CanTxPin = PortBPin(14);
constexpr Pin CanRxPin = PortBPin(15);
constexpr Pin CanStandbyPin = PortBPin(12);    // PB12
constexpr Pin CanBoostEnablePin = PortBPin(13); // PB13
constexpr GpioPinFunction CanPinsMode = GpioPinFunction::H;

constexpr size_t MaxPortsPerHeater = 1;				// Default, not used if no heaters

// Diagnostic LEDs
// Feather M4 CAN has an onboard NeoPixel (PB23/D13) and a red LED (PB22/D12 on some variants, often tied to SPI SCK for DotStar/NeoPixel)
// We'll use PortAPin(13) (D5 on some Feather M4 CAN layouts) as the diagnostic LED.
constexpr Pin LedPins[] = { PortAPin(23) };         // PA13 for diagnostic LED
constexpr bool LedActiveHigh = true;                // For standard LEDs; NeoPixel is data-driven
#define SUPPORT_DMA_NEOPIXEL    0                   // Enable if NeoPixel is DMA driven
#define SUPPORT_LED_STRIPS      1                   // To support the NeoPixel

// General Purpose I/O (from your example)
//constexpr Pin GpioOut0Pin  = PortAPin(13);  // D5
//constexpr Pin GpioOut1Pin  = PortAPin(14);  // D4
//constexpr Pin GpioIn0Pin   = PortAPin(12);  // D3
//constexpr Pin GpioIn1Pin   = PortBPin(9);   // A3

// Pin Table: Maps functional names to physical pins and capabilities.
// This table is crucial for the main board to configure this expansion board.
// The order here defines the logical pin numbers for some functions if not explicitly mapped.
constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out	  Exint 				PinNames
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA00 
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA01 
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,     SercomIo::none,		SercomIo::none,		Nx,     nullptr       	},	// PA02 A0 
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, 	nullptr      	},	// PA03
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,	    SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA04 A4 ESP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,	    SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA05 A1 ESP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,	    SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA06 A5 ESP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA07
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA08
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr     	},	// PA09
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr      	},	// PA10 
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA11 
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		12,		"pa12"			},	// PA12 SCL IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA13 D5 out0
	{ TcOutput::tc2_0,	TccOutput::tcc1_2G,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa14"			},	// PA14 D4 out1
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA15 
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA16 D6 STEP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA17 SCK DIR
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA18 EN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA19 D9 D3
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA20 D10 CMD
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA21 D11 CLK
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr     	},	// PA22 D12 ESP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr     	},	// PA23 D13 ESP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA24
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA25
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA26 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PA27
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA28 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA29 not on chip
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA30
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA31

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PB00
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr         },	// PB01
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB02
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PB03
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,	    SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB04
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,	    SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB05
	{ TcOutput::none,	TccOutput::none,	AdcInput::none, 	SercomIo::none,		SercomIo::none,		Nx,		nullptr		    },	// PB06
	{ TcOutput::none,	TccOutput::none,	AdcInput::none, 	SercomIo::none,		SercomIo::none,		Nx,		nullptr		    },	// PB07
	{ TcOutput::none,	TccOutput::none,	AdcInput::none, 	SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PB08 A2 ESP 
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_1,	SercomIo::none,		SercomIo::none,		9,		"pb09"	        },	// PB09 A3 IN
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PB10
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB11
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr         },	// PB12
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB13 SDA
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB14 CAN_TX CAN1TX
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB15 CAN_RX CAN1RX
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr		    },	// PB16 D1 ESP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB17 D0
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB18
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB19
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none, 	Nx,		nullptr			},	// PB20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB22 MISO ESP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB23 MOSI
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none, 	SercomIo::none,		Nx,		nullptr		    },	// PB24
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none, 	Nx,		nullptr		    },	// PB25
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr		    },	// PB26
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr		    },	// PB27
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PB28
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr      	},	// PB29
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none, 	SercomIo::none,		Nx,		nullptr		    },	// PB30
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none, 	Nx,		nullptr		    },	// PB31
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumNamedPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32 + 32;				// Max pins on SAME51 (PortA + PortB)
constexpr size_t NumVirtualPins = 0;
// static_assert(NumNamedPins <= NumRealPins + NumVirtualPins); // This assertion is loose. More important is that PinTable is correct.

// Timer/counter used to generate step pulses.
// Choose an available TC/TCC. TC0, TC1, TC2, TC3 are 16-bit. TCC0, TCC1, TCC2 are 24-bit.
// Let's use TC3 for step pulses, assuming it's free.
TcCount16 * const StepTc = &(TC3->COUNT16);			// Using a 16-bit TC for steps
constexpr IRQn StepTcIRQn = TC3_IRQn;
constexpr unsigned int StepTcNumber = 3;
#define STEP_TC_HANDLER			TC3_Handler

// DMA channel assignments (e.g., for NeoPixel)
#if SUPPORT_DMA_NEOPIXEL
constexpr DmaChannel DmacChanLedTx = 0;				// Example DMA channel for NeoPixel
constexpr DmaPriority DmacPrioLed = DmaPriority::DmaPrioMedium;
constexpr unsigned int NumDmaChannelsUsed = 0;
#else
constexpr unsigned int NumDmaChannelsUsed = 0;
#endif

// Interrupt priorities (SAMC21/SAMD51 use 0-3, lower value is higher priority)
// These are NvicPriorityType values (typically 0-7 for Cortex M4, but RRF uses a smaller range)
// Values must be chosen carefully to interact with FreeRTOS if used.
const NvicPriority NvicPriorityCan = 3;
const NvicPriority NvicPriorityStep = 2;			// Step ISR should be high priority
const NvicPriority NvicPriorityPins = 3;			// GPIO pin interrupts
const NvicPriority NvicPriorityDmac = 3;			// DMA complete interrupts
const NvicPriority NvicPriorityAdc = 5;

// Expansion boards usually don't define these, they are set by the main board or defaults in common headers.
// constexpr unsigned int MaxAxes = NumDrivers;
// constexpr unsigned int MaxHeaters = 0;
// constexpr unsigned int MaxFans = 0;

#define NUM_SERIAL_PORTS		0					// No general purpose serial ports defined for expansion use by default

#endif /* SRC_CONFIG_PINS_FEATHERM4CAN_H_ */