/*
 * Pins_FeatherM4CAN.h

 */


#ifndef SRC_CONFIG_PINS_FEATHERM4CAN_H_
#define SRC_CONFIG_PINS_FEATHERM4CAN_H_

#include <Hardware/PinDescription.h>


#define BOARD_TYPE_NAME			"FeatherM4CAN"
#define BOOTLOADER_NAME			"SAME5x"			

// General features
#define HAS_VREF_MONITOR		0					
#define HAS_VOLTAGE_MONITOR		0					
#define HAS_12V_MONITOR			0					
#define HAS_CPU_TEMP_SENSOR		1					
#define HAS_ADDRESS_SWITCHES	0					
#define HAS_BUTTONS				0		
#define USE_SERIAL_DEBUG		1			

// Drivers configuration
#define SUPPORT_DRIVERS			1
#define HAS_SMART_DRIVERS		0					
#define HAS_STALL_DETECT		0
#define SINGLE_DRIVER			1					
#define SUPPORT_SLOW_DRIVERS	1
#define DEDICATED_STEP_TIMER	1					
#define SUPPORT_INPUT_SHAPING	1					

#define ACTIVE_HIGH_STEP		1
#define ACTIVE_HIGH_DIR			1
#define ACTIVE_HIGH_ENABLE		0					

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			0
#define SUPPORT_TMC2208			0
#define SUPPORT_TMC2209			0
#define SUPPORT_TMC2240			0

constexpr size_t NumDrivers = 1;

PortGroup * const StepPio = &(PORT->Group[0]);
constexpr Pin StepPins[NumDrivers] = { PortAPin(16) };
constexpr Pin DirectionPins[NumDrivers] = { PortAPin(17) };
constexpr Pin EnablePins[NumDrivers] = { PortAPin(18) };


#define SUPPORT_THERMISTORS		0
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_DMA_NEOPIXEL    0 
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_LIS3DH			0
#define SUPPORT_LDC1612			0
#define SUPPORT_DHT_SENSOR		0

#define USE_MPU					0
#define USE_CACHE				1					

constexpr bool UseAlternateCanPins = true;			
constexpr Pin CanStandbyPin = PortBPin(12);    // PB12
constexpr Pin CanBoostEnablePin = PortBPin(13); // PB13

constexpr size_t MaxPortsPerHeater = 0;

constexpr Pin LedPins[] = { PortAPin(23) };
constexpr bool LedActiveHigh = true;
constexpr Pin NeoPixelPWR = PortBPin(3);

constexpr auto sercom0dPad0 = SercomIo::sercom0d + SercomIo::pad0;
constexpr auto sercom0dPad1 = SercomIo::sercom0d + SercomIo::pad1;
constexpr auto sercom2cPad0 = SercomIo::sercom2c + SercomIo::pad0;
constexpr auto sercom2cPad1 = SercomIo::sercom2c + SercomIo::pad1;
constexpr auto sercom5cPad0 = SercomIo::sercom5c + SercomIo::pad0;
constexpr auto sercom5cPad1 = SercomIo::sercom5c + SercomIo::pad1;
constexpr auto sercom5dPad0 = SercomIo::sercom5d + SercomIo::pad0;

constexpr PinDescription PinTable[] =
{
	//	TC					TCC					ADC					SERCOM in			SERCOM out	  Exint 				PinNames
	// Port A
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA00 32.768 CRYSTAL
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA01 32.768 CRYSTAL
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_0,   SercomIo::none,		SercomIo::none,		2,     "pa02"       	},	// PA02
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx, 	nullptr      	},	// PA03 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_4,   SercomIo::none,		sercom0dPad0,		4,		"pa04"			},	// PA04	
	{ TcOutput::tc0_1,	TccOutput::none,	AdcInput::adc0_5,   sercom0dPad1,		SercomIo::none,		5,		"pa05"			},	// PA05
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc0_6,	SercomIo::none,		SercomIo::none,		6,		"pa06"			},	// PA06
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA07 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA08 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr     	},	// PA09 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr      	},	// PA10 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA11 NC 
	{ TcOutput::none,	TccOutput::tcc0_6F,	AdcInput::none,		SercomIo::none,		sercom2cPad0,		12,		"pa12"			},	// PA12
	{ TcOutput::tc2_1,	TccOutput::none,	AdcInput::none,		sercom2cPad1,		SercomIo::none,		13,		"pa13"			},	// PA13
	{ TcOutput::none,	TccOutput::tcc2_0F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		14,		"pa14"			},	// PA14
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA15 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA16 STEP
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA17 DIR
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA18 ENA
	{ TcOutput::tc3_1,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		3,		"pa19"			},	// PA19
	{ TcOutput::none,	TccOutput::tcc1_4F,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa20"			},	// PA20
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa21"			},	// PA21
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		"pa22"	     	},	// PA22
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr     	},	// PA23 LED
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA24 USB D-
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA25 USB D+
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA26 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PA27 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA28 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA29 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA30 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PA31 NC

	// Port B
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PB00 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr         },	// PB01 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		sercom5dPad0,		Nx,		"led"			},	// PB02 NEOPIXEL
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PB03 NEOPIXEL POWER
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,	    SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB04 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,	    SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB05 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none, 	SercomIo::none,		SercomIo::none,		Nx,		nullptr		    },	// PB06 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none, 	SercomIo::none,		SercomIo::none,		Nx,		nullptr		    },	// PB07 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_0, 	SercomIo::none,		SercomIo::none,		8,		nullptr	        },	// PB08
	{ TcOutput::none,	TccOutput::none,	AdcInput::adc1_1,	SercomIo::none,		SercomIo::none,		9,		nullptr	        },	// PB09
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PB10 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB11 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr         },	// PB12 CAN STANDBY
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB13 CAN BOOST
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB14 CAN_TX CAN1TX
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB15 CAN_RX CAN1RX
	{ TcOutput::none,	TccOutput::tcc3_0F,	AdcInput::none,		SercomIo::none,		sercom5cPad0,		0,		"pb16"		    },	// PB16
	{ TcOutput::none,	TccOutput::tcc3_1F,	AdcInput::none,		sercom5cPad1,		SercomIo::none,		1,		"pb17"			},	// PB17
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB18 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB19 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none, 	Nx,		nullptr			},	// PB20 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB21 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB22 CRYSTAL
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr			},	// PB23 CRYSTAL
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none, 	SercomIo::none,		Nx,		nullptr		    },	// PB24 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none, 	Nx,		nullptr		    },	// PB25 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr		    },	// PB26 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr		    },	// PB27 NC 
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr	        },	// PB28 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none,		Nx,		nullptr      	},	// PB29 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none, 	SercomIo::none,		Nx,		nullptr		    },	// PB30 NC
	{ TcOutput::none,	TccOutput::none,	AdcInput::none,		SercomIo::none,		SercomIo::none, 	Nx,		nullptr		    },	// PB31 NC
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
constexpr size_t NumNamedPins = ARRAY_SIZE(PinTable);
constexpr size_t NumRealPins = 32 + 32;
constexpr size_t NumVirtualPins = 0;

// Timer/counter used to generate step pulses and other sub-millisecond timings
TcCount32 * const StepTc = &(TC4->COUNT32);
constexpr IRQn StepTcIRQn = TC4_IRQn;
constexpr unsigned int StepTcNumber = 4;
#define STEP_TC_HANDLER			TC4_Handler

// Available UART ports
#define NUM_SERIAL_PORTS		1
constexpr IRQn Serial0_IRQn = SERCOM2_0_IRQn;

// DMA channel assignments
constexpr DmaChannel DmacChanTmcTx = 0;
constexpr DmaChannel DmacChanTmcRx = 1;
constexpr DmaChannel DmacChanAdc0Rx = 2;
constexpr DmaChannel DmacChanLedTx = 3;

constexpr unsigned int NumDmaChannelsUsed = 4;			// must be at least the number of channels used, may be larger. Max 12 on the SAME5x.

constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 3;
constexpr DmaPriority DmacPrioAdcRx = 2;
constexpr DmaPriority DmacPrioLed = 1;

// Interrupt priorities, lower means higher priority. 0-2 can't make RTOS calls.
const NvicPriority NvicPriorityStep = 3;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 3;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 3;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityCan = 4;
const NvicPriority NvicPriorityDmac = 5;				// priority for DMA complete interrupts
const NvicPriority NvicPriorityAdc = 5;

#endif /* SRC_CONFIG_PINS_FEATHERM4CAN_H_ */