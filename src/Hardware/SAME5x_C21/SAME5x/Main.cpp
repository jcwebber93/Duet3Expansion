#include <CoreIO.h>

#if SAME5x

void AppInit() noexcept
{
	// We use the standard clock configuration, so nothing needed here
}

// Return the XOSC frequency in MHz
unsigned int AppGetXoscFrequency() noexcept
{
#ifdef EXP3HC
	return 0;		// auto detect 12 or 25MHz
#else
	return 25;		// other boards (only EXP1HCL at present) always use 25MHz
#endif
}

// Return the XOSC number
unsigned int AppGetXoscNumber() noexcept
{
#ifdef FeatherM4CAN
	return 1; // Indicates XOSC1 is used
#else
	return 0;
#endif
}

// Return the CPU frequency in MHz (100 for -MF 100MHz variant, 120 for standard SAME5x)
unsigned int AppGetCpuFrequency() noexcept
{
#if defined(DP3EXB)
	return 100;
#else
	return 120;
#endif
}

// Return the XOSC startup delay value for OSCCTRL XOSCCTRL STARTUP field
unsigned int AppGetXoscStartup() noexcept
{
#if defined(DP3EXB)
	return 0xA;			// ~31ms startup for marginal DP3EXB crystal
#else
	return 0;
#endif
}

#endif

// End
