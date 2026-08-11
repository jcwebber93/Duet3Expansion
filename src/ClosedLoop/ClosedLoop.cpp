/*
 * ClosedLoop.cpp
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

/*
 * Some observations on closed loop control of motors:
 * 1. We can generate steps in a step ISR as we do in open loop ode, then add each microstep to the desired position.
 *    But it is probably better to calculate the position directly from the move parameters.
 *    We can calculate the current velocity and acceleration as well, which should enable more accurate positioning.
 * 2. Currently we allow the user to set microstepping as usual, because that works in both open and closed loop mode.
 *    If we required that the users uses microstepping equal to the encoder resolution, then it would be difficult to operate in open loop mode.
 * 3. We can write the control loop to aim for a particular encoder reading (by first converting the required step position to an encoder reading),
 *    or we can convert encoder readings to step positions and aim for a particular step position. If we do the latter then the control loop
 *    will typically hunt between two encoder positions that straddle the required step position. We could perhaps avoid that by treating
 *    a motor position error that is less than about 0.6 encoder steps as zero error. However, targeting encoder count has the advantage that
 *    we could use mostly integer maths, which would be better if we want to implement this on SAMC21 or RP2040 platforms.
 * 3. We need to be sure that we can count encoder steps and motor positions without loss of accuracy when they get high.
 *    The encoder with greatest resolution is currently the AS5047, which has 16384 counts/rev = 81.92 counts/full step. TLI5012B has the same resolution.
 *    If the machine has 100 microsteps/mm at x16 microstepping then this corresponds to 512 counts/mm. If stored as a 32-bit signed integer
 *    it will overflow at about +/-4.2km which should be OK for axes but perhaps not for extruders.
 * 4. If we represent motor step position as a float then to be accurate to the nearest encoder count we have only 24 bits available,
 *    so the highest we can go without loss of resolution is +/- 32.768 metres.
 * 5. Since all motion commands sent to the board are relative, we could reset the step count and encoder count periodically by a whole number of revolutions,
 *    as long as we are careful to do it atomically. If/when we support linear encoders, this will not be necessary for those.
 * 6. When using an absolute encoder, although it could be treated like a relative encoder for the purposes of motor control,
 *    the angle information may be useful to compensate for leadscrew nut irregularities; so we should preserve the angle information.
 */

#include "ClosedLoop.h"

#if SUPPORT_CLOSED_LOOP

using std::atomic;
using std::numeric_limits;

# include "Encoders/AbsoluteRotaryEncoder.h"
# include "Encoders/QuadratureEncoderPdec.h"
# include "Encoders/LinearCompositeEncoder.h"

# include <ClosedLoop/DerivativeAveragingFilter.h>

# include <math.h>
# include <Platform/Platform.h>
# include <Movement/Move.h>
# include <Heating/Heat.h>				// for NewDriverFault()
# include <General/Bitmap.h>
# include <Platform/TaskPriorities.h>
# include <CAN/CanInterface.h>
# include <CanMessageBuffer.h>
# include <CanMessageFormats.h>
# include <CanMessageGenericParser.h>
# include <CanMessageGenericTables.h>
# include <AppNotifyIndices.h>
# include <hri_gclk_e54.h>
# include <AnalogOut.h>

# if SUPPORT_TMC51xx || SUPPORT_TMC2240_SPI
#  include "Movement/StepperDrivers/TMC51xx.h"
# endif

#define BASIC_TUNING_DEBUG	0

constexpr size_t DataCollectionTaskStackWords = 200;		// Size of the stack for the data collection task
constexpr size_t EncoderCalibrationTaskStackWords = 500;	// Size of the stack for the encoder calibration task

SampleBuffer ClosedLoop::sampleBuffer;												// buffer for collecting samples - shared between all drives if we have more than one

// Tasks and task loops
static Task<DataCollectionTaskStackWords> *dataTransmissionTask = nullptr;			// Data transmission task - handles sending back the buffered sample data
static Task<EncoderCalibrationTaskStackWords> *encoderCalibrationTask = nullptr;	// Encoder calibration task - handles calibrating the encoder in the background

extern "C" [[noreturn]] void DataTransmissionTaskEntry(void *param) noexcept
{
	((ClosedLoop*)param)->DataTransmissionTaskLoop();
}

extern "C" [[noreturn]] void EncoderCalibrationTaskEntry(void *param) noexcept
{
	((ClosedLoop*)param)->EncoderCalibrationTaskLoop();
}

// Helper function to convert a time period (expressed in StepTimer::Ticks) to ms
static inline float TickPeriodToMillis(StepTimer::Ticks tickPeriod) noexcept
{
	return tickPeriod * StepTimer::StepClocksToMillis;
}

// Helper function to cat all the current tuning errors onto a reply in human-readable form
void ClosedLoop::ReportTuningErrors(TuningErrors tuningErrorBitmask, const StringRef &reply) noexcept
{
	if (tuningErrorBitmask & TuningError::NeedsBasicTuning) 			{ reply.cat(", the drive has not had basic tuning done"); }
	if (tuningErrorBitmask & TuningError::NotCalibrated) 				{ reply.cat(", the drive has not been calibrated"); }
	if (tuningErrorBitmask & TuningError::SystemError) 					{ reply.cat(", a system error occurred while tuning"); }
	if (tuningErrorBitmask & TuningError::TuningOrCalibrationInProgress){ reply.cat(", encoder calibration is in progress"); }
	if (tuningErrorBitmask & TuningError::InconsistentMotion)			{ reply.cat(", the measured motion was inconsistent"); }
	if (tuningErrorBitmask & TuningError::TooLittleMotion)				{ reply.cat(", the measured motion was less than expected"); }
	if (tuningErrorBitmask & TuningError::TooMuchMotion)				{ reply.cat(", the measured motion was more than expected"); }
}

void ClosedLoop::SetTargetToCurrentPosition() noexcept
{
	const float multiplier = (moveInstance->GetDirectionValueNoCheck(driverNumber)) ? 1.0f : -1.0f;
	const float physicalSteps = (float)encoder->GetCurrentCount() * encoder->GetStepsPerCount();
	mParams.position = physicalSteps * multiplier;
	moveInstance->SetCurrentMotorSteps(driverNumber, mParams.position);		// expects logical, converts to physical internally
	// ResetDriveMovementState() seeds DriveMovement::positionAtMoveStart/positionAtSegmentStart/
	// currentMotorPosition directly from this value with no further sign adjustment of its own
	// (see DriveMovement::ResetState()) - GetCurrentMotion() later reconstructs mParams.position
	// straight from those same fields, so this must be given the same signed (multiplied)
	// convention as mParams.position above, not the raw unmultiplied physicalSteps. Passing the
	// unmultiplied value here caused the very next GetCurrentMotion() re-fetch to silently
	// overwrite the correctly-signed snap on line 107 with a wrong-signed one.
	moveInstance->ResetDriveMovementState(driverNumber, mParams.position);
}

// Set the motor currents and update desiredStepPhase
// The phase is normally in the range 0 to 4095 but when tuning it can be 0 to somewhat over 8192.
// We must take it modulo 4096 when computing the currents. Function Trigonometry::FastSinCos does that.
// 'magnitude' must be in range 0.0..1.0
void ClosedLoop::SetMotorPhase(uint16_t phase, float magnitude) noexcept
{
	desiredStepPhase = phase;
	float sine, cosine;
	Trigonometry::FastSinCos(phase, sine, cosine);
	coilA = (int16_t)lrintf(cosine * magnitude);
	coilB = (int16_t)lrintf(sine * magnitude);

# if (SUPPORT_TMC51xx || SUPPORT_TMC2240_SPI) && SINGLE_DRIVER
	SmartDrivers::SetMotorPhases(driverNumber, (((uint32_t)(uint16_t)coilB << 16) | (uint32_t)(uint16_t)coilA) & 0x01FF01FF);
# elif !SUPPORT_DCSERVO
#  error Multi driver code not implemented
# endif
// DC servo: coil phase control not applicable; torque applied via SetDcPwm()
}

#if SAME5x && (SUPPORT_TMC51xx || SUPPORT_TMC2240_SPI)
static_assert(TmcClockGclkNumber == GclkNumApp1 || TmcClockGclkNumber == GclkNumApp2);	// check that this GCLK number has been reserved for application use
#endif
#if SUPPORT_DCSERVO

void ClosedLoop::InitDcPwm() noexcept
{
	// Some of the items below likely aren't needed, as they were generated while troubleshooting a complete loss of PWM output on the pins. Needs to be reverted and retested.
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1;
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2;


	hri_gclk_write_PCHCTRL_reg(GCLK, TCC1_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum48MHz) | GCLK_PCHCTRL_CHEN);
	hri_gclk_write_PCHCTRL_reg(GCLK, TCC2_GCLK_ID, GCLK_PCHCTRL_GEN(GclkNum48MHz) | GCLK_PCHCTRL_CHEN);


	SetPinFunction(DcServoFwdPin, GpioPinFunction::F);	// TCC2/WO[2]
	SetPinFunction(DcServoRevPin, GpioPinFunction::G);	// TCC1/WO[2]


	AnalogOut::Write(DcServoFwdPin, 0.0f);
	AnalogOut::Write(DcServoRevPin, 0.0f);
}

void ClosedLoop::SetDcPwm(float controlSignal) noexcept
{
	// The control signal is in the range -256.0 to +256.0.
	// A positive signal means forward motion, negative means reverse.
	const float pwmDuty = constrain<float>(fabsf(controlSignal) / 256.0f, 0.0f, 1.0f);

	if (controlSignal > 0.0f)
	{
		// Forward motion
		AnalogOut::Write(DcServoFwdPin, pwmDuty);
		AnalogOut::Write(DcServoRevPin, 0.0f);
	}
	else if (controlSignal < 0.0f)
	{
		// Backward motion
		AnalogOut::Write(DcServoFwdPin, 0.0f);
		AnalogOut::Write(DcServoRevPin, pwmDuty);
	}
	else
	{
		// Stop
		AnalogOut::Write(DcServoFwdPin, 0.0f);
		AnalogOut::Write(DcServoRevPin, 0.0f);
	}
}
#endif

#if SUPPORT_TMC51xx || SUPPORT_TMC2240_SPI
//static_assert(ClockGenGclkNumber == GclkClosedLoop);							// check that this GCLK number has been reserved

static void GenerateTmcClock()
{
	// Currently we program DPLL0 to generate 120MHz output, so to get 15MHz with 1:1 ratio we divide by 8.
	// We could divide by 7 instead giving 17.143MHz with 25ns and 33.3ns times. TMC2160A max is 18MHz, minimum 16ns and 16ns low.
	// Max SPI clock frequency is half this clock frequency.
#if STM32
	qq;	//TODO
#else
	ConfigureGclk(TmcClockGclkNumber, GclkSource::dpll0, 8, true);
	SetPinFunction(TmcClockPin, TmcClockPinPeriphMode);
#endif
	SmartDrivers::SetTmcExternalClock(15000000);
}
#endif

// Module initialisation
/*static*/ void ClosedLoop::Init() noexcept
{
#if SUPPORT_TMC51xx
	GenerateTmcClock();															// generate the clock for the TMC2160A
#endif
}

void ClosedLoop::InitInstance() noexcept
{
	SetPinMode(EncoderCsPin, OUTPUT_HIGH);										// make sure that any attached SPI encoder is not selected

	// Initialise to default error thresholds
	errorThresholds[0] = DefaultClosedLoopPositionWarningThreshold;
	errorThresholds[1] = DefaultClosedLoopPositionErrorThreshold;

	PIDITerm = 0.0;
	errorDerivativeFilter.Reset();
	speedFilter.Reset();

	UpdateStandstillCurrent();

	// Set up the data transmission task
	dataTransmissionTask = new Task<DataCollectionTaskStackWords>;
	dataTransmissionTask->Create(DataTransmissionTaskEntry, "CLSend", this, TaskPriority::ClosedLoopDataTransmission);
}

GCodeResult ClosedLoop::ProcessM569Point1(CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	// Set default parameters
	uint8_t tempEncoderType = GetEncoderType().ToBaseType();
	float tempCPR;
	float tempKp = Kp;
	float tempKi = Ki;
	float tempKd = Kd;
	float tempKv = Kv;
	float tempKa = Ka;
	float tempKpp = Kpp;
	uint16_t tempStepsPerRev = 200;
	size_t numThresholds = 2;
	float tempErrorThresholds[numThresholds];
	float tempTorquePerAmp = torquePerAmp;
#if SUPPORT_DCSERVO
	uint8_t tempDcOutputMode = (uint8_t)dcOutputMode;
	float tempDcMaxCurrentTmc = dcMaxCurrentTmc;
	uint8_t tempDcTmcPhaseSelect = dcTmcPhaseSelect;
#endif
#if SUPPORT_FOC
	uint8_t tempPolePairCount = polePairCount;
	float tempFocSupplyVoltage = focSupplyVoltage;
	float tempFocVoltageLimit = focVoltageLimit;
#endif

	// Pull changed parameters
	const bool seenT = parser.GetUintParam('T', tempEncoderType);
	const bool seenC = parser.GetFloatParam('C', tempCPR);
	const bool seenPid = parser.GetFloatParam('R', tempKp) | parser.GetFloatParam('I', tempKi)  | parser.GetFloatParam('D', tempKd) | parser.GetFloatParam('J', tempKpp)
						| parser.GetFloatParam('V', tempKv) | parser.GetFloatParam('A', tempKa);
	const bool seenE = parser.GetFloatArrayParam('E', numThresholds, tempErrorThresholds);
	const bool seenS = parser.GetUintParam('S', tempStepsPerRev);
	// Q is torque-per-amp only. It used to be shared with a FOC velocity limit; see ClosedLoop.h for why
	// that limit was removed.
	const bool seenQ = parser.GetFloatParam('Q', tempTorquePerAmp);
#if SUPPORT_DCSERVO
	const bool seenU = parser.GetUintParam('U', tempDcOutputMode);
	const bool seenZ = parser.GetUintParam('Z', tempDcTmcPhaseSelect);
#endif
#if SUPPORT_FOC
	const bool seenL = parser.GetUintParam('L', tempPolePairCount);
	float tempFocMaxTorque = focMaxTorque;
	const bool seenN = parser.GetFloatParam('N', tempFocSupplyVoltage);
	const bool seenO = parser.GetFloatParam('O', tempFocVoltageLimit);
	// Current-mode (d/q) loops. F is also the on/off switch: zero means voltage mode, which is the
	// default and what every configuration without current sensing must use.
	float tempFocCurrentKp = focCurrentKp, tempFocCurrentKi = focCurrentKi, tempFocMaxCurrent = focMaxCurrent;
	// One array parameter rather than three letters, because M569.1's table had no room for three - see
	// the note at the end of CanMessageGenericTables.h. F{Kp, Ki, maxAmps}; F0:0:0 returns to voltage mode.
	float tempCurrentPid[3] = { tempFocCurrentKp, tempFocCurrentKi, tempFocMaxCurrent };
	size_t numCurrentPid = 3;
	const bool seenCurrentPid = parser.GetFloatArrayParam('F', numCurrentPid, tempCurrentPid);
	if (seenCurrentPid)
	{
		if (numCurrentPid != 3)
		{
			reply.copy("F (FOC current loop) needs three values: F{proportional gain, integral gain, max amps}");
			return GCodeResult::error;
		}
		tempFocCurrentKp = tempCurrentPid[0];
		tempFocCurrentKi = tempCurrentPid[1];
		tempFocMaxCurrent = tempCurrentPid[2];
	}
#endif
#if SUPPORT_DCSERVO && SUPPORT_FOC
	// W is shared: route to FOC max-torque when motor type is a FOC type, DC max-current otherwise.
	// Read W once regardless, then assign below after seenT is processed.
	float tempW = 0.0f;
	const bool seenW = parser.GetFloatParam('W', tempW);
	const bool seenFocW = seenW && (motorType == EncoderType::bldc || motorType == EncoderType::stepperFoc || motorType == EncoderType::hybridStepperFoc
								 || (seenT && (tempEncoderType == (uint32_t)EncoderType::bldc || tempEncoderType == (uint32_t)EncoderType::stepperFoc || tempEncoderType == (uint32_t)EncoderType::hybridStepperFoc)));
	if (seenW) { if (seenFocW) { tempFocMaxTorque = tempW; } else { tempDcMaxCurrentTmc = tempW; } }
#elif SUPPORT_DCSERVO
	const bool seenW = parser.GetFloatParam('W', tempDcMaxCurrentTmc);
#elif SUPPORT_FOC
	const bool seenFocW = parser.GetFloatParam('W', tempFocMaxTorque);
#endif

	// Report back if no parameters to change
	if (!(seenT || seenC || seenPid || seenE || seenQ || seenS
#if SUPPORT_DCSERVO
		|| seenU || seenW || seenZ
#endif
#if SUPPORT_FOC
		|| seenL || seenFocW || seenN || seenO || seenCurrentPid
#endif
	)) {
		if (encoder == nullptr)
		{
			reply.cat("No encoder configured");
		}
		else
		{
			reply.catf("Encoder type: %s", GetEncoderType().ToString());
			encoder->AppendStatus(reply);
			reply.lcatf("PID parameters P=%.1f I=%.3f D=%.3f V=%.1f A=%.1f, torque constant %.2fNm/A, J=%.1f",
						(double)Kp, (double)Ki, (double)Kd, (double)Kv, (double)Ka, (double)torquePerAmp, (double)Kpp);
			reply.lcatf("Warning/error threshold %.2f/%.2f", (double)errorThresholds[0], (double)errorThresholds[1]);
#if SUPPORT_DCSERVO
			if (isDcServoMode)
			{
				reply.lcatf(", DC output: %s", (dcOutputMode == DcServoOutputMode::IoxPwm) ? "IOX" :
													(dcTmcPhaseSelect == 0) ? "TMC (phase A)" : "TMC (phase B)");
				if (dcOutputMode == DcServoOutputMode::TmcSinglePhase) {
					reply.catf(", max current %.2fA", (double)dcMaxCurrentTmc);
				}
			}
#endif
#if SUPPORT_FOC
			if (motorType == EncoderType::bldc || motorType == EncoderType::stepperFoc || motorType == EncoderType::hybridStepperFoc)
			{
				if (focSupplyVoltage > 0.0f && focVoltageLimit > 0.0f)
				{
					reply.lcatf("FOC voltage limit %.2fV of %.2fV nominal supply (torque scale %.3f)", (double)focVoltageLimit, (double)focSupplyVoltage, (double)GetFocVoltageScale());
				}
				else
				{
					reply.lcat("FOC voltage limit not configured (N and O not set) - torqueMagnitude applied unscaled");
				}
				reply.lcatf("FOC commutation: %" PRIu32 " counts/elecRev (%s), encoder direction %+d",
					GetFocCountsPerElecRev(), (focCountsPerElecRev != 0) ? "measured" : "from C/L", (int)focEncoderDirection);
			}
#endif
		}
		return GCodeResult::ok;
	}

	if (seenT && !seenC && (tempEncoderType == EncoderType::rotaryQuadrature || tempEncoderType == EncoderType::linearComposite))
	{
		reply.copy("Missing C parameter");
		return GCodeResult::error;
	}

	if ((seenC || seenS) && !seenT)
	{
		reply.copy("M569.1 C and S parameters not permitted without T parameter");
		return GCodeResult::error;
	}

	// Validate the new params
	if (tempEncoderType >= EncoderType::NumValues)
	{
		reply.copy("Invalid T value. Valid values are ");
		for (size_t i = 1; i < EncoderType::NumValues; ++i) { reply.catf("%s%u", (i == 1) ? "" : ", ", i); }
		return GCodeResult::error;
	}
	if (seenE && (tempErrorThresholds[0] < 0 || tempErrorThresholds[1] < 0))
	{
		reply.copy("Error threshold value must nor be less than zero");
		return GCodeResult::error;
	}
	if (seenC && tempCPR < 2 * tempStepsPerRev && tempEncoderType != EncoderType::linearComposite)
	{
		reply.copy("Encoder counts/rev must be at least two times steps/rev");
		return GCodeResult::error;
	}
	if (seenQ && tempTorquePerAmp <= 0.0)
	{
		reply.copy("Torque per amp must be positive");
		return GCodeResult::error;
	}
#if SUPPORT_DCSERVO
	if (seenU && tempDcOutputMode > (uint8_t)DcServoOutputMode::TmcSinglePhase)
	{
		reply.copy("Invalid U parameter. 0=IOX, 1=TMC");
		return GCodeResult::error;
	}
	if (seenW && tempDcMaxCurrentTmc <= 0.0)
	{
		reply.copy("W (max DC current) must be positive");
		return GCodeResult::error;
	}
	if (seenZ && tempDcTmcPhaseSelect > 1)
	{
		reply.copy("Z (TMC phase) must be 0 or 1");
		return GCodeResult::error;
	}
#endif
#if SUPPORT_FOC
	if (seenFocW && (tempFocMaxTorque <= 0.0f || tempFocMaxTorque > 1.0f))
	{
		reply.copy("W (FOC max torque fraction) must be in range (0.0, 1.0]");
		return GCodeResult::error;
	}
	// Zero would divide by zero when deriving counts per electrical revolution, which then feeds the
	// commutation angle - reject it here rather than producing a garbage angle at runtime.
	if (seenL && tempPolePairCount == 0)
	{
		reply.copy("L (pole pair count) must be at least 1");
		return GCodeResult::error;
	}
	if (seenN && tempFocSupplyVoltage <= 0.0f)
	{
		reply.copy("N (nominal supply voltage) must be positive");
		return GCodeResult::error;
	}
	if (seenO && tempFocVoltageLimit <= 0.0f)
	{
		reply.copy("O (FOC voltage limit) must be positive");
		return GCodeResult::error;
	}
	if (seenO && !seenN && focSupplyVoltage <= 0.0f)
	{
		reply.copy("O (FOC voltage limit) requires N (nominal supply voltage) to be set, either now or previously");
		return GCodeResult::error;
	}
	if (seenCurrentPid)
	{
		if (tempFocCurrentKp < 0.0f || tempFocCurrentKi < 0.0f || tempFocMaxCurrent < 0.0f)
		{
			reply.copy("F (FOC current loop) values must not be negative");
			return GCodeResult::error;
		}
		// Catch the half-configured case rather than silently staying in voltage mode: asking for current
		// control and getting voltage control, with no indication, is exactly the kind of thing that gets
		// diagnosed as a tuning problem.
		if (tempFocCurrentKp > 0.0f && tempFocMaxCurrent <= 0.0f)
		{
			reply.copy("F: a non-zero current loop gain requires a non-zero maximum current, e.g. F0.02:25:2.0");
			return GCodeResult::error;
		}
	}
#endif
#if SUPPORT_DCSERVO
	if (seenT && tempEncoderType == EncoderType::dcServo)
	{
		// For DC servo control, we force a 1:1 mapping of steps to quadrature encoder counts.
		// The user's M92 steps/mm should be set to the encoder's quadrature counts/mm.
		tempStepsPerRev = (uint16_t)(tempCPR * 4);
	}
#endif
#if SUPPORT_FOC
	if (seenT && (tempEncoderType == (uint8_t)EncoderType::bldc
				|| tempEncoderType == (uint8_t)EncoderType::stepperFoc
				|| tempEncoderType == (uint8_t)EncoderType::hybridStepperFoc)
			&& seenC)
	{
		// For FOC motors, force a 1:1 mapping of steps to quadrature encoder counts, matching the DC servo convention.
		// Set M92 to the encoder's quadrature counts per unit of travel (CPR * 4 per revolution).
		tempStepsPerRev = (uint16_t)(tempCPR * 4);
	}
#endif

	// Drop out of closed loop before touching the encoder or the DC output mode, because the control
	// loop task dereferences the encoder object on every ~80us tick and we are about to delete it.
	//
	// NB the DC-output-mode condition below used to be written as a braceless `if` on the line directly
	// above the `if (seenT)` block, which made that block its BODY. With SUPPORT_DCSERVO enabled (see
	// SAMME51.h) that meant the driver was only ever dropped to open loop when U was ALSO supplied and
	// changed - so a plain `M569.1 P... T... C...` deleted and reallocated the encoder underneath a
	// still-running closed-loop control loop, and swapped the commutation scale factor live with no
	// re-alignment. Both conditions now share one properly-braced statement.
	if (seenT
#if SUPPORT_DCSERVO
		|| (seenU && (DcServoOutputMode)tempDcOutputMode != dcOutputMode)
#endif
	   )
	{
		SetClosedLoopEnabled(ClosedLoopMode::open, reply);
	}

	// Set the new closed loop parameters
	{
		TaskCriticalSectionLocker lock;			// don't allow the closed loop task to see an inconsistent combination of these values

		if (seenPid)
		{
			Kp = tempKp;
			Ki = tempKi;
			Kd = tempKd;
			Kv = tempKv;
			Ka = tempKa;
			Kpp = tempKpp;
			PIDITerm = 0.0;
			errorDerivativeFilter.Reset();
			speedFilter.Reset();
		}

		if (seenE)
		{
			errorThresholds[0] = tempErrorThresholds[0];
			errorThresholds[1] = tempErrorThresholds[1];
		}

		if (seenQ)
		{
			torquePerAmp = tempTorquePerAmp;
		}

#if SUPPORT_DCSERVO
		if (seenU || seenW || seenZ)
		{
			dcOutputMode = (DcServoOutputMode)tempDcOutputMode;
			// Enforce the hard-coded maximum current limit for safety
			dcMaxCurrentTmc = constrain<float>(tempDcMaxCurrentTmc, 0.0f, MaxDcServoTmcCurrent);
			dcTmcPhaseSelect = tempDcTmcPhaseSelect;
	#if SUPPORT_TMC51xx
			if (dcOutputMode == DcServoOutputMode::TmcSinglePhase) {
				SmartDrivers::SetDriverMode(driverNumber, (unsigned int)DriverMode::direct);
				moveInstance->EnableDrive(driverNumber);
			}
#endif
		}
#endif
#if SUPPORT_FOC
		if (seenL)
		{
			polePairCount = tempPolePairCount;
		}
		if (seenFocW)
		{
			focMaxTorque = tempFocMaxTorque;
		}
		if (seenN)
		{
			focSupplyVoltage = tempFocSupplyVoltage;
		}
		if (seenO)
		{
			focVoltageLimit = tempFocVoltageLimit;
		}
		if (seenCurrentPid)
		{
			focCurrentKp = tempFocCurrentKp;
			focCurrentKi = tempFocCurrentKi;
			focMaxCurrent = tempFocMaxCurrent;
			// Retuning invalidates whatever the integrators had accumulated under the old gains.
			focIdIntegral = 0.0f;
			focIqIntegral = 0.0f;
		}
#endif
		if (seenT)
		{
			isDcServoMode = (tempEncoderType == EncoderType::dcServo);
		}
	}


	if (seenT)
	{
		// We set the mode to open loop earlier in this function so no need to do it here
		DeleteObject(encoder);

		// If the magnetic encoder type was provided, check that it is valid
		MagneticEncoderType magEncoderType(MagneticEncoderType::as5047d);
		{
			String<StringLength20> magneticEncoderTypeString;
			if (parser.GetStringParam('Y', magneticEncoderTypeString.GetRef()))
			{
				magEncoderType = MagneticEncoderType(magneticEncoderTypeString.c_str());
				if (!magEncoderType.IsValid())
				{
					reply.printf("unrecognised magnetic encoder type '%s'", magneticEncoderTypeString.c_str());
					return GCodeResult::error;
				}
			}
		}

		switch (tempEncoderType)
		{
		case EncoderType::none:
		default:
			// encoder is already nullptr
			break;

		case EncoderType::rotaryMagnetic:
			encoder = CreateRotaryEncoder(magEncoderType, tempStepsPerRev, *Platform::sharedSpi, EncoderCsPin);
			CreateCalibrationTask();
			break;

#if SUPPORT_COMPOSITE_ENCODER
		case EncoderType::linearComposite:
			encoder = new LinearCompositeEncoder(tempCPR, tempStepsPerRev, *Platform::sharedSpi, EncoderCsPin, magEncoderType);
			CreateCalibrationTask();
			break;
#endif

//#if SUPPORT_QUADRATURE_ENCODER
		case EncoderType::rotaryQuadrature:
			encoder = new QuadratureEncoderPdec((uint32_t)tempCPR, tempStepsPerRev);
			break;

#if SUPPORT_DCSERVO
		case EncoderType::dcServo:
			encoder = new QuadratureEncoderPdec((uint32_t)tempCPR, tempStepsPerRev);
			InitDcPwm();
			break;
#endif

#if SUPPORT_FOC
		case EncoderType::bldc:
		case EncoderType::hybridStepperFoc:
			// 3-phase PWM output: BLDC uses full SVPWM; hybrid stepper uses 2-phase cos/sin + synthetic C offset.
			// Underlying position encoder is quadrature (seenC) or absolute magnetic.
			{
				if (seenC)
				{
					encoder = new QuadratureEncoderPdec((uint32_t)tempCPR, tempStepsPerRev);
				}
				else
				{
					encoder = CreateRotaryEncoder(magEncoderType, tempStepsPerRev, *Platform::sharedSpi, EncoderCsPin);
					CreateCalibrationTask();
				}
				DeleteObject(focController);
				const FocOutputMode focMode = (tempEncoderType == EncoderType::hybridStepperFoc)
												? FocOutputMode::HybridStepper
												: FocOutputMode::ThreePhase;
				focController = new FocController(FocPhaseUPin, FocPhaseVPin, FocPhaseWPin,
													FocPhaseUFn, FocPhaseVFn, FocPhaseWFn, focMode);
				focController->Init(FocPwmFrequency);
				InitFocDriverFaultPin();
				motorType = (EncoderType)tempEncoderType;
#if SUPPORT_DRV8316_SPI
				if (drv8316 == nullptr)
				{
					drv8316 = new DRV8316(Platform::GetDrv8316Spi(), Drv8316CsPin);
					drv8316->Init();
					// Claim and configure ADC0 for the DRV8316's current-sense outputs. Done here rather
					// than at board init so it only happens on a drive that actually has the hardware.
					FocCurrentSense::Init();
				}
#endif
			}
			break;

# if SUPPORT_FOC_STEPPER
		case EncoderType::stepperFoc:
			// 4PWM 2-phase stepper via sign-magnitude H-bridge (e.g. L298N).
			{
				if (seenC)
				{
					encoder = new QuadratureEncoderPdec((uint32_t)tempCPR, tempStepsPerRev);
				}
				else
				{
					encoder = CreateRotaryEncoder(magEncoderType, tempStepsPerRev, *Platform::sharedSpi, EncoderCsPin);
					CreateCalibrationTask();
				}
				DeleteObject(focController);
				focController = new FocController(FocStepperIn1Pin, FocStepperIn2Pin,
													FocStepperIn3Pin, FocStepperIn4Pin,
													FocStepperIn1Fn, FocStepperIn2Fn,
													FocStepperIn3Fn, FocStepperIn4Fn,
													FocStepperEnaPin, FocStepperEnbPin);
				focController->Init(FocStepperPwmFrequency);
				motorType = EncoderType::stepperFoc;
			}
			break;
# endif
#endif
		}

		if (encoder != nullptr)
		{
			const GCodeResult rslt = encoder->Init(reply);
			if (rslt <= GCodeResult::warning)
			{
				tuningError = isDcServoMode ? 0 : encoder->MinimalTuningNeeded();
				encoder->LoadLUT(tuningError);
			}
			else
			{
				DeleteObject(encoder);
			}
			return rslt;
		}
		else if (tempEncoderType != EncoderType::none)
		{
			reply.printf("unsupported encoder type %u", (unsigned int)tempEncoderType);
			return GCodeResult::error;
		}
	}

	return GCodeResult::ok;
}

// M569.4 Set torque mode
GCodeResult ClosedLoop::ProcessM569Point4(CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	float requestedTorque;
	if (!parser.GetFloatParam('T', requestedTorque))
	{
		reply.copy("missing T parameter");
		return GCodeResult::error;
	}
	float maxSpeed = torqueModeMaxSpeed;
	float rawMaxSpeed;
	if (parser.GetFloatParam('V', rawMaxSpeed))
	{
		maxSpeed = rawMaxSpeed/StepTimer::StepClockRate;		// convert to full steps per step clock
	}

	if (currentMode == ClosedLoopMode::open || tuning != 0 || tuningError != 0)
	{
		reply.copy("torque mode not available when driver is in open loop mode, has not been tuned, or is being tuned");
		return GCodeResult::error;
	}

	{
		TaskCriticalSectionLocker lock;

		if (requestedTorque == 0.0)								// if asking to exit torque mode
		{
			if (inTorqueMode)
			{
				ExitTorqueMode();
			}
			return GCodeResult::ok;
		}

		if (!hasMovementCommand)
		{
			moveInstance->EnableDrive(driverNumber);			// enable the drive if it isn't already enabled
			torqueModeDirection = (moveInstance->GetDirectionValueNoCheck(driverNumber) == (requestedTorque > 0.0));
#if SUPPORT_TMC51xx
			torqueModeCommandedCurrentFraction = min<float>(fabsf(requestedTorque)/(torquePerAmp * SmartDrivers::GetCurrent(driverNumber) * 0.001), 1.0);
#else
			torqueModeCommandedCurrentFraction = min<float>(fabsf(requestedTorque), 1.0f);
#endif
			torqueModeMaxSpeed = maxSpeed;
			inTorqueMode = true;
			return GCodeResult::ok;
		}
	}

	reply.copy("cannot enter torque mode while moving");
	return GCodeResult::error;
}

GCodeResult ClosedLoop::ProcessM569Point5(const CanMessageStartClosedLoopDataCollection& msg, const StringRef& reply) noexcept
{
	if (encoder == nullptr)
	{
		reply.copy("No encoder has been configured");
		return GCodeResult::error;
	}

	if (CollectingData())
	{
		reply.copy("Driver is already collecting data");
		return GCodeResult::error;
	}

	uint8_t requestedMode;
	if (msg.movement != 0)
	{
		requestedMode = (uint8_t)RecordingMode::OnNextMove;		// when recording a tuning move we ignore the mode and start 5ms before it
	}
	else
	{
		requestedMode = msg.mode + 1;							// the A parameter is out of step with the enumeration by 1
		if (requestedMode != (uint8_t)RecordingMode::Immediate && requestedMode != (uint8_t)RecordingMode::OnNextMove)
		{
			reply.copy("Invalid recording mode");
			return GCodeResult::error;
		}
	}

	if (msg.movement != 0 && tuning != 0)
	{
		reply.copy("Driver is already performing tuning");
		return GCodeResult::error;
	}

	// Belt and braces against the packing loop in DataTransmissionTaskLoop(), which writes one whole
	// sample before it checks whether the next one fits. The main board rejects an over-budget filter
	// before it ever gets here, but an older or third-party host would not, and the consequence would be
	// a write past CanMessageClosedLoopData::data[] rather than a clean failure.
	if (!ClosedLoopSampleFits(msg.filter))
	{
		reply.printf("Requested variables need %u bytes per sample, limit is %u",
						(unsigned int)ClosedLoopSampleLength(msg.filter), (unsigned int)MaxClosedLoopSampleBytes);
		return GCodeResult::error;
	}

	{
		TaskCriticalSectionLocker lock;

		// Set up the recording vars
		filterRequested = msg.filter;
		sampleBuffer.Init(ClosedLoopSampleLength(filterRequested));
		sampleBufferOverflowed = false;
		samplesRequested = msg.numSamples;
		samplesSent = 0;
		samplesCollected = 0;
		dataCollectionIntervalTicks = (msg.rate == 0) ? 1 : StepTimer::StepClockRate/msg.rate;
		dataCollectionStartTicks = whenNextSampleDue = StepTimer::GetMovementTimerTicks();
		samplingMode = (RecordingMode)requestedMode;				// do this one last, it triggers data collection

		StartTuning(msg.movement);
	}
	return GCodeResult::ok;
}

GCodeResult ClosedLoop::ProcessM569Point6(CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	if (encoder == nullptr)
	{
		reply.copy("no encoder configured");
		return GCodeResult::error;
	}

	uint8_t desiredTuning;
	if (!parser.GetUintParam('V', desiredTuning))
	{
		// We have been called to return the status after the previous call returned "not finished"
		if (tuning != 0)
		{
			return GCodeResult::notFinished;
		}

		// If we were checking the calibration, report the result
		return (basicTuningDataReady) ? ProcessBasicTuningResult(reply) : ProcessCalibrationResult(reply);
	}

	switch (desiredTuning)
	{
	default:
		reply.copy("invalid tuning mode");
		return GCodeResult::error;

	case 1:		// basic calibration
		if (!encoder->UsesBasicTuning())
		{
			reply.copy("basic tuning is not applicable to absolute encoders");
			return GCodeResult::error;
		}
		break;

	case 2:
	case 3:
	case 4:
		if (!encoder->UsesCalibration())
		{
			reply.copy("calibration is not applicable to the configured encoder type");
			return GCodeResult::error;
		}
		if (desiredTuning == 4)
		{
			// Tuning move 4 just clears the lookup table
			encoder->ScrubLUT();
			reply.copy("Encoder calibration cleared");
			tuningError |= TuningError::NotCalibrated;
			return GCodeResult::ok;
		}
		break;

	case 64:
		break;
	}

	// Here if this is a new command to start a tuning move
	// Check we are in direct drive mode
#if SUPPORT_TMC51xx
	if (SmartDrivers::GetDriverMode(driverNumber) != DriverMode::direct)
	{
		reply.copy("Driver is not in direct mode");
		return GCodeResult::error;
	}
#endif

	if (!moveInstance->EnableIfIdle(driverNumber))
	{
		reply.copy("Driver is not enabled");
		return GCodeResult::error;
	}

	StartTuning(desiredTuning);
	return GCodeResult::notFinished;
}

bool ClosedLoop::OkayToSetDriverIdle() const noexcept
{
	//TODO should we forbid idle current in closed loop and assisted open loop modes too?
	return !inTorqueMode;
}

// Update the standstill current fraction for this drive.
void ClosedLoop::UpdateStandstillCurrent() noexcept
{
#if SINGLE_DRIVER
# if SUPPORT_TMC51xx
	holdCurrentFraction = SmartDrivers::GetStandstillCurrentPercent(driverNumber) * 0.01;
# else
	holdCurrentFraction = 0.0;
# endif
#else
# error Multi driver code not implemented
#endif
}

// This is called when tuning has finished and the basicTuningDataReady flag is set
GCodeResult ClosedLoop::ProcessBasicTuningResult(const StringRef& reply) noexcept
{
	// Tuning has finished - there are now 3 scenarios
	// 1. No tuning errors exist (!tuningError)							= OK
	// 2. No new tuning errors exist !(~prevTuningError & tuningError)	= WARNING
	// 3. A new tuning error has been introduced (else)					= WARNING
	basicTuningDataReady = false;
	reply.printf("Driver %u.0 basic tuning ", CanInterface::GetCanAddress());

	const TuningErrors newTuningErrors = encoder->ProcessTuningData();
	if (newTuningErrors != 0)
	{
		tuningError &= ~TuningError::TuningOrCalibrationInProgress;
		reply.cat("failed");
		tuningError = (tuningError & ~(TuningError::TooMuchMotion | TuningError::TooLittleMotion | TuningError::InconsistentMotion)) | newTuningErrors;
		ReportTuningErrors(newTuningErrors, reply);
		if (newTuningErrors & (TuningError::TooMuchMotion | TuningError::TooLittleMotion))
		{
			reply.catf(", measured counts/rev is about %.1f", (double)((encoder->GetMeasuredCountsPerStep() * encoder->GetStepsPerRev()) * 0.25));
		}
		return GCodeResult::error;
	}

	PIDITerm = 0.0;
	errorDerivativeFilter.Reset();
	speedFilter.Reset();
	SetTargetToCurrentPosition();
	tuningError &= ~TuningError::TuningOrCalibrationInProgress;

	const float hyst = encoder->GetMeasuredHysteresis();
	const unsigned int increaseFactor = (encoder->GetType() == EncoderType::linearComposite) ? LinearEncoderIncreaseFactor : 1;

	// DC 2025-02-14: if it's linear composite encoder then the backlash measured by the quadrature encoder isn't critical because we're not using it for commutation
	if (hyst >= MaxSafeBacklash * increaseFactor && encoder->GetType() != EncoderType::linearComposite)
	{
		tuningError = TuningError::HysteresisTooHigh;
		reply.catf("failed, measured backlash (%.3f step) is too high", (double)hyst);
		return GCodeResult::error;
	}
	else if (hyst >= MaxGoodBacklash * increaseFactor)
	{
		reply.catf("succeeded but measured backlash (%.3f step) is high", (double)hyst);
		tuningError = 0;
		return GCodeResult::warning;
	}
	else
	{
		reply.catf("succeeded, measured backlash %.3f step", (double)hyst);
		tuningError = 0;
		return GCodeResult::ok;
	}
}

// This function is run by the encoder calibration task.
// Its purpose is to wait for encoder calibration data to become available and process it.
// Processing it takes several seconds, so we need to do it in a separate task to avoid the main board timing out awaiting CAN responses.
void ClosedLoop::EncoderCalibrationTaskLoop() noexcept
{
	for (;;)
	{
		TaskBase::TakeIndexed(NotifyIndices::ClosedLoopDataTransmission);
		if (encoder != nullptr && encoder->UsesCalibration() && calibrationState == CalibrationState::dataReady)
		{
			calibrationErrors = encoder->Calibrate(calibrateNotCheck);
			calibrationState = CalibrationState::complete;
		}
	}
}

void ClosedLoop::CreateCalibrationTask() noexcept
{
	if (encoderCalibrationTask == nullptr)
	{
		encoderCalibrationTask = new Task<EncoderCalibrationTaskStackWords>;
		encoderCalibrationTask->Create(EncoderCalibrationTaskEntry, "EncCal", this, TaskPriority::SpinPriority);		// must be same priority as main task
	}
}

GCodeResult ClosedLoop::ProcessCalibrationResult(const StringRef& reply) noexcept
{
	if (calibrationState == CalibrationState::dataReady)
	{
		// Waiting for the calibration task to finish processing the calibration via our Spin() function
		return GCodeResult::notFinished;
	}

	reply.printf("Driver %u.0 calibration ", CanInterface::GetCanAddress());
	if (calibrationState != CalibrationState::complete)
	{
		reply.cat("failed (no reason available)");
		return GCodeResult::error;
	}

	// Must have calibrationState == CalibrationState::complete
	calibrationState = CalibrationState::notReady;
	if (!calibrateNotCheck)
	{
		reply.cat("check ");
	}
	if (calibrationErrors != 0)
	{
		reply.cat("failed");
		if (calibrateNotCheck)
		{
			tuningError = (tuningError & ~(TuningError::TooMuchMotion | TuningError::TooLittleMotion | TuningError::InconsistentMotion | TuningError::TuningOrCalibrationInProgress)) | calibrationErrors;
		}
		else
		{
			tuningError &= ~TuningError::TuningOrCalibrationInProgress;
		}
		ReportTuningErrors(calibrationErrors, reply);
		if (calibrationErrors & (TuningError::TooMuchMotion | TuningError::TooLittleMotion))
		{
			reply.catf(", measured counts/step is about %.1f", (double)encoder->GetMeasuredCountsPerStep());
		}
		return GCodeResult::error;
	}

	PIDITerm = 0.0;
	errorDerivativeFilter.Reset();
	speedFilter.Reset();
	SetTargetToCurrentPosition();

	const float hyst = encoder->GetMeasuredHysteresis();
	if (hyst >= MaxSafeBacklash)
	{
		if (calibrateNotCheck) { tuningError = TuningError::HysteresisTooHigh; }
		else { tuningError &= ~TuningError::TuningOrCalibrationInProgress; }
		reply.catf("failed, measured backlash (%.3f step) is too high", (double)hyst);
		return GCodeResult::error;
	}
	else if (hyst >= MaxGoodBacklash)
	{
		if (calibrateNotCheck) { tuningError = 0; }
		else { tuningError &= ~TuningError::TuningOrCalibrationInProgress; }
		reply.catf("succeeded but measured backlash (%.3f step) is high", (double)hyst);
	}
	else
	{
		if (calibrateNotCheck) { tuningError = 0; }
		else { tuningError &= ~TuningError::TuningOrCalibrationInProgress; }
		reply.catf("succeeded, measured backlash is %.3f step", (double)hyst);
	}

	// Report the calibration errors and corrections
	reply.lcatf("%s encoder reading errors: ", (calibrateNotCheck) ? "Original" : "Residual");
	encoder->AppendCalibrationErrors(reply);
	if (calibrateNotCheck)
	{
		reply.lcatf("Corrections made: ");
		encoder->AppendLUTCorrections(reply);
	}

	return GCodeResult::ok;
}

void ClosedLoop::StartTuning(uint8_t tuningMode) noexcept
{
	if (tuningMode != 0)
	{
		whenLastTuningStepTaken = StepTimer::GetMovementTimerTicks() + stepTicksBeforeTuning;	// delay the start to allow brake release and motor current buildup
		tuning = (tuningMode == 1) ? BASIC_TUNING_MANOEUVRE
					: (tuningMode == 2) ? ENCODER_CALIBRATION_MANOEUVRE
						: (tuningMode == 3) ? ENCODER_CALIBRATION_CHECK
							: (tuningMode == 64) ? STEP_MANOEUVRE
								: 0;
	}
}

// Call this when we have stopped basic tuning movement and are ready to switch to closed loop control
void ClosedLoop::FinishedBasicTuning() noexcept
{
	basicTuningDataReady = true;
	tuningError |= TuningError::TuningOrCalibrationInProgress;				// to prevent movement until we are done tuning
}

// Call this when encoder calibration has finished collecting data
void ClosedLoop::ReadyToCalibrate(bool store) noexcept
{
	calibrateNotCheck = store;
	if (encoderCalibrationTask != nullptr)
	{
		calibrationState = CalibrationState::dataReady;
		tuningError |= TuningError::TuningOrCalibrationInProgress;			// to prevent movement in case we are re-calibrating
		encoderCalibrationTask->Give(NotifyIndices::ClosedLoopDataTransmission);
	}
}

// This is called by tuning to execute a step
void ClosedLoop::AdjustTargetMotorSteps(float amount) noexcept
{
	mParams.position += amount;
	moveInstance->SetCurrentMotorSteps(driverNumber, lrintf(mParams.position));
}

void ClosedLoop::InstanceControlLoop(StepTimer::Ticks now, StepTimer::Ticks timeElapsed) noexcept
{
	if (encoder == nullptr)
	{
		return;
	}

	// Poll DRV8316 fault registers at ~10 ms intervals (125 ticks × 80 µs = 10 ms)
#if SUPPORT_DRV8316_SPI
	if (drv8316 != nullptr)
	{
		if (++drv8316PollCounter >= 125u)
		{
			drv8316PollCounter = 0;
			drv8316->PollFaults();
		}
	}
#endif

	// Read the current state of the drive.
	if (encoder->TakeReading())
	{
		// Per-drive-type preparation only. Everything after this - the control law, stall detection,
		// sample collection and statistics - is shared.
		//
		// This used to be three parallel branches, each with its own copy of the control call, sample
		// collection and statistics block, and each falling through into the shared tail as well. The
		// result was that DC servo and FOC ran the whole tail a second time per tick: for DC servo
		// (whose tuningError is forced to 0) that meant ControlMotorCurrents() executed TWICE per tick
		// with an identical timestamp, double-integrating the I term and corrupting the D term, and for
		// both it meant the statistics were double-counted. Meanwhile stall detection lived only in the
		// tail, behind `tuningError == 0`, so FOC - which permanently carries NeedsBasicTuning from its
		// relative encoder - never had any stall detection at all.
		//
		// Basic tuning calibrates a stepper's pole-pair phase against the encoder. It does not apply to
		// DC servo (no phases to calibrate) or to FOC (commutation comes from the L pole-pair count and
		// the alignment measurement), so mask NeedsBasicTuning out of the gate for those - otherwise a
		// quadrature encoder's initial tuning error blocks the control loop forever.
		TuningErrors effectiveTuningError = tuningError;
		if (isDcServoMode
#if SUPPORT_FOC
			|| IsFocMode()
#endif
		   )
		{
			// DC servo and FOC fetch motion parameters and compute currentPositionError inside
			// ControlMotorCurrents(), so there is nothing to prepare here.
			effectiveTuningError &= ~TuningError::NeedsBasicTuning;
		}
		else
		{
			// Calculate and store the current error in full steps
			const bool hadMovementCommand = hasMovementCommand;
			hasMovementCommand = moveInstance->GetCurrentMotion(driverNumber, now, mParams);
			if (hasMovementCommand)
			{
				// If this is the start of a new move sequence, we must resynchronise the target to the current position.
				// This is because the main board's Move class will have reset its position to zero at the start of a new move,
				// but the physical motor and encoder are still at the end of the last move.
				if (!hadMovementCommand)
				{
					SetTargetToCurrentPosition();
					moveInstance->GetCurrentMotion(driverNumber, now, mParams); // Re-fetch motion parameters based on the corrected position
				}
				if (inTorqueMode)
				{
					ExitTorqueMode();
				}
				if (samplingMode == RecordingMode::OnNextMove)
				{
					dataCollectionStartTicks = whenNextSampleDue = now;
					samplingMode = RecordingMode::Immediate;
				}
			}

			const float targetEncoderReading = rintf(mParams.position * encoder->GetCountsPerStep());
			currentPositionError = (float)(targetEncoderReading - encoder->GetCurrentCount()) * encoder->GetStepsPerCount();
			errorDerivativeFilter.ProcessReading(currentPositionError, now);
			speedFilter.ProcessReading(encoder->GetCurrentCount() * encoder->GetStepsPerCount(), now);
		}
		float currentFraction = 0.0;
		if (currentMode != ClosedLoopMode::open)
		{
			if (tuning != 0)														// if we need to tune, do it
			{
				// Limit the rate at which we command tuning steps. We need to do signed comparison because initially, whenLastTuningStepTaken is in the future.
				const int32_t timeSinceLastTuningStep = (int32_t)(now - whenLastTuningStepTaken);
				if (timeSinceLastTuningStep >= (int32_t)stepTicksPerTuningStep)
				{
					whenLastTuningStepTaken = now;
					PerformTune();
				}
				else if (samplingMode == RecordingMode::OnNextMove && timeSinceLastTuningStep + (int32_t)DataCollectionIdleStepTicks >= 0)
				{
					dataCollectionStartTicks = whenNextSampleDue = now;
					samplingMode = RecordingMode::Immediate;
				}
			}
			else if (effectiveTuningError == 0)
			{
				const bool hadMovementCommand = hasMovementCommand;
				currentFraction = ControlMotorCurrents(now, timeElapsed); // otherwise control those motor currents!
				// DC servo and FOC only learn about a new move inside ControlMotorCurrents(), so the
				// OnNextMove trigger has to be checked here for them. For a classic stepper the
				// preparation block above has already done it, and hasMovementCommand cannot have
				// changed since, so this is a no-op.
				if (samplingMode == RecordingMode::OnNextMove && hasMovementCommand && !hadMovementCommand)
				{
					dataCollectionStartTicks = whenNextSampleDue = now;
					samplingMode = RecordingMode::Immediate;
				}
				UpdateStallDetection();
			}
		}

		// Collect a sample, if we need to
		if (samplingMode == RecordingMode::Immediate && (int32_t)(now - whenNextSampleDue) >= 0)
		{
			// It's time to take a sample
			CollectSample();
			whenNextSampleDue += dataCollectionIntervalTicks;
		}

		// Update the statistics
		TaskCriticalSectionLocker lock;						// prevent a race with the Heat task that sends the statistics

		const float absPositionError = fabsf(currentPositionError);
		if (absPositionError > periodMaxAbsPositionError)
		{
			periodMaxAbsPositionError = absPositionError;
		}
		periodSumOfPositionErrorSquares += fsquare(currentPositionError);
		if (currentFraction > periodMaxCurrentFraction)
		{
			periodMaxCurrentFraction = currentFraction;
		}
		periodSumOfCurrentFractions += currentFraction;
		++periodNumSamples;
	}
}

// Update the stall/pre-stall flags from the current position error.
// Called once per control tick for every drive type, immediately after the control law has run (so
// currentPositionError is fresh). Note the control law is NOT gated on !stall: the position error has to
// keep being updated while a stall is latched, or the flag could never clear again. The stop action below
// therefore fires on the detection edge only - it is the driver fault reported to the main board that
// actually stops the move.
void ClosedLoop::UpdateStallDetection() noexcept
{
	if (inTorqueMode)
	{
		stall = preStall = false;
		return;
	}

	const float positionErr = fabsf(currentPositionError);
	if (stall)
	{
		// Reset the stall flag when the position error falls to below half the tolerance, to avoid generating too many stall events
		//TODO do we need a minimum delay before resetting too?
		if (errorThresholds[1] <= 0 || positionErr < errorThresholds[1]/2)
		{
			stall = false;
			if (isDcServoMode
#if SUPPORT_FOC
				|| IsFocMode()
#endif
			   )
			{
				PIDITerm = 0.0f;
				last_vel_error = 0.0f;
				last_filtered_D = 0.0f;
				speedFilter.Reset();
			}
		}
	}
	else
	{
		stall = errorThresholds[1] > 0 && positionErr > errorThresholds[1];
		if (stall)
		{
			// A stall has just been detected. Drop the drive output and clear the accumulators so the
			// integrator does not wind up while the fault is being handled. A classic stepper is
			// deliberately left energised - its phase currents are its holding torque - but a DC servo or
			// a FOC drive would otherwise keep pushing at whatever torque it stalled at.
#if SUPPORT_DCSERVO
			if (isDcServoMode)
			{
				PIDITerm = 0.0f;
				last_vel_error = 0.0f;
				last_filtered_D = 0.0f;
				ApplyDcTorque(0.0);
			}
#endif
#if SUPPORT_FOC
			if (IsFocMode())
			{
				PIDITerm = 0.0f;
				last_vel_error = 0.0f;
				last_filtered_D = 0.0f;
				ApplyFocTorque(0.0f, lastFocElectricalAngle);
			}
#endif
			Heat::NewDriverFault();
		}
		else
		{
			preStall = errorThresholds[0] > 0 && positionErr > errorThresholds[0];
		}
	}
}

// Send data from the buffer to the main board over CAN
[[noreturn]] void ClosedLoop::DataTransmissionTaskLoop() noexcept
				{
	while (true)
	{
		const RecordingMode locMode = samplingMode;										// to capture the volatile variable
		if (locMode == RecordingMode::Immediate || locMode == RecordingMode::SendingData)
		{
			// Started a new data collection
			samplesSent = 0;

			// Loop until everything has been read. Stop when either we have sent the requested number of samples, or the state is SendingData and we have sent all the data in the buffer.
			// Note, this may mean that the last packet contains no data and has the "last" flag set.
			bool finished;
			do
			{
				// Set up a CAN message
				CanMessageBuffer buf;
				CanMessageClosedLoopData& msg = *(buf.SetupRequestMessageNoRid<CanMessageClosedLoopData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress()));

				// Populate the control fields
				msg.firstSampleNumber = samplesSent;
				msg.ClearReservedFields();
				msg.filter = filterRequested;

				unsigned int numSamplesInMessage = 0;
				size_t dataIndex = 0;
				do
				{
					while (samplesSent == samplesCollected && samplingMode == RecordingMode::Immediate)
					{
						TaskBase::TakeIndexed(NotifyIndices::ClosedLoopDataTransmission);			// wait for data to be available
					}

					if (samplesSent < samplesCollected)
					{
						dataIndex += sampleBuffer.GetSample(msg.data + dataIndex);
						++samplesSent;								// update this one first to avoid a race condition
						++numSamplesInMessage;
					}
					finished = (samplingMode != RecordingMode::Immediate && samplesSent == samplesCollected);
				} while (!finished && dataIndex + sampleBuffer.GetBytesPerSample() <= ARRAY_SIZE(msg.data));

				msg.numSamples = numSamplesInMessage;
				msg.lastPacket = finished;
				msg.overflowed = sampleBufferOverflowed;
				msg.badSample = sampleBuffer.HadBadSample();

				// Send the CAN message
				buf.dataLength = msg.GetActualDataLength(dataIndex);
				CanInterface::Send(&buf);
			} while (!finished);
			samplingMode = RecordingMode::None;
		}
		else
		{
			TaskBase::TakeIndexed(NotifyIndices::ClosedLoopDataTransmission);	// wait for a new data collection to start
		}
	}
}

// Store a sample in the buffer
void ClosedLoop::CollectSample() noexcept
{
	if (sampleBuffer.IsFull())
	{
		sampleBufferOverflowed = true;							// the buffer is full so tell the sending task about it
		samplingMode = RecordingMode::SendingData;				// stop collecting data
	}
	else
	{
		sampleBuffer.PutF32(TickPeriodToMillis(StepTimer::GetMovementTimerTicks() - dataCollectionStartTicks));		// always collect this

		// Logical-space PID values (and, below, measured/target position) are sign-flipped when S0
		// direction is active. Apply the direction multiplier consistently to ALL of them so every
		// chart trace is plotted in the same sense: when the control loop is converging, Measured
		// Motor Steps visually tracks Target Motor Steps 1:1 regardless of which physical rotation
		// direction (clockwise/counterclockwise) S0/S1 or the motor's encoder wiring happen to
		// correspond to. Without this, Measured was logged in raw physical (encoder) space while the
		// control loop's actual convergence criterion (currentPositionError, which DOES get this
		// multiplier) is in direction-adjusted logical space - so a perfectly-tracking motor could
		// show Target and Measured moving in visually opposite directions on the tuning graph, making
		// a correctly-functioning drive look like it was spinning the wrong way.
#if SUPPORT_DCSERVO && SUPPORT_FOC
		const float recordMultiplier = isDcServoMode ? dcServoMultiplier
								: (motorType == EncoderType::bldc || motorType == EncoderType::stepperFoc || motorType == EncoderType::hybridStepperFoc) ? focMultiplier
								: 1.0f;
#elif SUPPORT_DCSERVO
		const float recordMultiplier = isDcServoMode ? dcServoMultiplier : 1.0f;
#elif SUPPORT_FOC
		const float recordMultiplier = (motorType == EncoderType::bldc || motorType == EncoderType::stepperFoc || motorType == EncoderType::hybridStepperFoc) ? focMultiplier : 1.0f;
#else
		constexpr float recordMultiplier = 1.0f;
#endif

		if (filterRequested & CL_RECORD_RAW_ENCODER_READING) 	{ sampleBuffer.PutI32(encoder->GetCurrentCount()); }
		if (filterRequested & CL_RECORD_CURRENT_MOTOR_STEPS) 	{ sampleBuffer.PutF32((float)encoder->GetCurrentCount() * encoder->GetStepsPerCount() * recordMultiplier); }
		// mParams.position is already in the same direction-adjusted convention as
		// CL_RECORD_CURRENT_MOTOR_STEPS above for every drive type: DC servo and FOC/BLDC never had a
		// stepper-style direction multiplier applied to it (Move::GetCurrentMotion() explicitly skips
		// that for both - see IsDcServo()/IsFoc() there), and for a classic stepper it already carries
		// the microstep-shift-adjusted multiplier from that same function. No further scaling needed.
		if (filterRequested & CL_RECORD_TARGET_MOTOR_STEPS) 	{ sampleBuffer.PutF32(mParams.position); }
		if (filterRequested & CL_RECORD_CURRENT_ERROR) 			{ sampleBuffer.PutF32(currentPositionError * recordMultiplier); }
		if (filterRequested & CL_RECORD_PID_CONTROL_SIGNAL)  	{ sampleBuffer.PutF16(PIDControlSignal * recordMultiplier); }
		if (filterRequested & CL_RECORD_PID_P_TERM)				{ sampleBuffer.PutF16(PIDPTerm * recordMultiplier); }
		if (filterRequested & CL_RECORD_PID_I_TERM)				{ sampleBuffer.PutF16(PIDITerm * recordMultiplier); }
		if (filterRequested & CL_RECORD_PID_D_TERM)				{ sampleBuffer.PutF16(PIDDTerm * recordMultiplier); }
		if (filterRequested & CL_RECORD_CURRENT_STEP_PHASE)  	{ sampleBuffer.PutU16(encoder->GetCurrentPhasePosition()); }
		if (filterRequested & CL_RECORD_DESIRED_STEP_PHASE)		{ sampleBuffer.PutU16(desiredStepPhase); }
		// NOTE: this channel carries vel_measured (steps per step-clock tick), NOT a stepper phase shift.
		// It is the one deliberate exception to "every channel reports what its name says" - the FOC and
		// DC servo cascades have no phase shift to report, and having measured velocity next to the
		// velocity setpoint terms is worth more than the name being accurate. Everything else in this
		// function was un-repurposed once the commutation bugs were fixed and the ad-hoc FOC debug
		// channels were no longer needed; per-sample FOC state now has no home here, see M122 instead.
		if (filterRequested & CL_RECORD_PHASE_SHIFT)  			{ sampleBuffer.PutF16(vel_measured * recordMultiplier); }
		if (filterRequested & CL_RECORD_COIL_A_CURRENT)			{ sampleBuffer.PutI16(coilA); }
		if (filterRequested & CL_RECORD_COIL_B_CURRENT)			{ sampleBuffer.PutI16(coilB); }
		if (filterRequested & CL_RECORD_PID_V_TERM)				{ sampleBuffer.PutF16(PIDVTerm * recordMultiplier); }
		if (filterRequested & CL_RECORD_PID_A_TERM)  			{ sampleBuffer.PutF16(PIDATerm * recordMultiplier); }
		if (filterRequested & CL_RECORD_PID_J_TERM)				{ sampleBuffer.PutF16(PIDJTerm * recordMultiplier); }
		if (filterRequested & CL_RECORD_MEASURED_VELOCITY)
		{
#if SUPPORT_DCSERVO || SUPPORT_FOC
			if (isDcServoMode
# if SUPPORT_FOC
				|| motorType == EncoderType::bldc || motorType == EncoderType::stepperFoc || motorType == EncoderType::hybridStepperFoc
# endif
			)
			{
				// Convert velocity from counts/tick to mm/sec for human-readable charting.
				const float vel_mm_per_sec = (vel_measured * recordMultiplier * StepTimer::StepClockRate) / moveInstance->DriveStepsPerMm(driverNumber);
				sampleBuffer.PutF16(vel_mm_per_sec);
			}
			else
#endif
			{
				sampleBuffer.PutF16(vel_measured);
			}
		}

		// FOC current-mode channels (bits 17-23). These must stay AFTER CL_RECORD_MEASURED_VELOCITY
		// (bit 16) - the write order here is the wire order, and the main board reads fields back in bit
		// order. Getting this wrong shifts every subsequent column in the CSV by one field, which shows
		// up as plausible-looking but wrong numbers rather than an obvious failure.
		//
		// Note on signs: none of these seven take recordMultiplier, unlike the position and PID channels
		// above. They are all electrical quantities describing what the windings are doing, and that does
		// not change with which way the axis is declared to turn - Ia/Ib/Ic obviously so, and the d/q pairs
		// because they are just those same currents resolved onto the rotor's own axes.
		//
		// An earlier draft of this comment proposed applying the multiplier to Iq alone, on the grounds
		// that its sign follows commanded torque. That is wrong, and the reason is worth keeping: Vq is
		// derived from the same commanded torque, so with a reversed axis (multiplier -1) Iq would print
		// with the opposite sign to the Vq that produced it, and the pair would look like a drive fighting
		// itself. Keeping the whole quadruple in the physical rotor frame keeps them comparable, which is
		// the entire reason for logging them together.
		{
			float ia = 0.0f, ib = 0.0f, ic = 0.0f;
#if SUPPORT_DRV8316_SPI
			FocCurrentSense::GetPhaseCurrents(ia, ib, ic);
#endif
			if (filterRequested & CL_RECORD_PHASE_CURRENT_A)		{ sampleBuffer.PutF16(ia); }
			if (filterRequested & CL_RECORD_PHASE_CURRENT_B)		{ sampleBuffer.PutF16(ib); }
			if (filterRequested & CL_RECORD_PHASE_CURRENT_C)		{ sampleBuffer.PutF16(ic); }
			if (filterRequested & CL_RECORD_CURRENT_D)			{ sampleBuffer.PutF16(lastFocId); }
			if (filterRequested & CL_RECORD_CURRENT_Q)			{ sampleBuffer.PutF16(lastFocIq); }
			if (filterRequested & CL_RECORD_VOLTAGE_D)			{ sampleBuffer.PutF16(lastFocVd); }
			if (filterRequested & CL_RECORD_VOLTAGE_Q)			{ sampleBuffer.PutF16(lastFocVq); }
		}

		sampleBuffer.FinishSample();
		++samplesCollected;
		if (samplesCollected == samplesRequested)
		{
			samplingMode = RecordingMode::SendingData;			// stop collecting data
		}
	}

	dataTransmissionTask->Give(NotifyIndices::ClosedLoopDataTransmission);
}

// Control the motor phase currents, returning the fraction of maximum current that we commanded
inline float ClosedLoop::ControlMotorCurrents(StepTimer::Ticks now, StepTimer::Ticks ticksSinceLastCall) noexcept
{
#if SUPPORT_DCSERVO
	if (isDcServoMode)
	{
		const float multiplier = (moveInstance->GetDirectionValueNoCheck(driverNumber)) ? 1.0f : -1.0f;
		dcServoMultiplier = multiplier;									// persist for CollectSample to convert logical→physical space

		// For DC servo, we must fetch motion parameters here because the main loop bypasses it for this encoder type.
		const bool hadMovementCommand = hasMovementCommand;
		hasMovementCommand = moveInstance->GetCurrentMotion(driverNumber, now, mParams);
		if (hasMovementCommand && !hadMovementCommand)
		{
			// Start of a new move, resynchronise target to current position
			SetTargetToCurrentPosition();
			// Reset velocity loop state to avoid transient spikes from stale history
			PIDITerm = 0.0f;
			last_vel_error = 0.0f;
			last_filtered_D = 0.0f;
			speedFilter.Reset();
			moveInstance->GetCurrentMotion(driverNumber, now, mParams); // Re-fetch
		}

		// Convert logical target back to physical counts for comparison, then error back to logical space
		const float targetPhysicalCount = (mParams.position * encoder->GetCountsPerStep()) * multiplier;
		currentPositionError = (targetPhysicalCount - (float)encoder->GetCurrentCount()) * encoder->GetStepsPerCount() * multiplier;
		speedFilter.ProcessReading(encoder->GetCurrentCount() * encoder->GetStepsPerCount(), now);

		const float timeDelta = (float)ticksSinceLastCall * (1.0/(float)StepTimer::StepClockRate);

		// 1. Outer Position Loop (P-controller)
		PIDJTerm = Kpp * currentPositionError;

		// 2. Feedforward Terms
		PIDVTerm = Kv * mParams.speed;
		PIDATerm = Ka * mParams.acceleration * (float)ticksSinceLastCall;	// steps/tick² * ticks = steps/tick, same units as vel_target

		// 3. Calculate Target Velocity (A feedforward is part of the velocity setpoint, not a raw torque bypass)
		const float vel_target = PIDJTerm + PIDVTerm + PIDATerm;

		// 4. Inner Velocity Loop (PID)
		// Velocity must also be converted to logical space for the PID comparison.
		// multiplier is 1.0 for S1 (forward) and -1.0 for S0 (reverse).
		vel_measured = speedFilter.GetDerivative() * multiplier;
		const float vel_error = vel_target - vel_measured;
		PIDPTerm = Kp * vel_error;
		PIDITerm = constrain<float>(PIDITerm + (Ki * timeDelta * 0.5f * (vel_error + last_vel_error)), -PIDIlimit, PIDIlimit);
		const float rawD = (timeDelta > 0.0f) ? (vel_error - last_vel_error) / timeDelta : last_filtered_D;
		last_filtered_D += 0.1f * (rawD - last_filtered_D);
		PIDDTerm = Kd * last_filtered_D;

		// 5. Final Control Signal
		PIDControlSignal = PIDPTerm + PIDITerm + PIDDTerm;
		// Apply the multiplier to the output to ensure torque direction matches the logical coordinate space.
		ApplyDcTorque(constrain<float>((PIDControlSignal * multiplier) / 256.0f, -1.0f, 1.0f));
		last_vel_error = vel_error;
		return fabsf(PIDControlSignal) / 256.0f;
	}
#endif

#if SUPPORT_FOC
	if (motorType == EncoderType::bldc
		|| motorType == EncoderType::stepperFoc
		|| motorType == EncoderType::hybridStepperFoc)
	{
		lastFocDriverFault = ReadFocDriverFault();						// sample every tick, before any mode-specific logic/early return, so a fault is caught the instant it happens
#if SUPPORT_DRV8316_SPI
		FocCurrentSense::Poll();										// collect the conversion the PWM carrier triggered, advance to the next phase
#endif

		const float multiplier = (moveInstance->GetDirectionValueNoCheck(driverNumber)) ? 1.0f : -1.0f;
		focMultiplier = multiplier;									// persist for CollectSample to convert logical→physical space

		const bool hadMovementCommand = hasMovementCommand;
		hasMovementCommand = moveInstance->GetCurrentMotion(driverNumber, now, mParams);
		if (hasMovementCommand && !hadMovementCommand)
		{
			// Start of a new move: resynchronise planner origin to current encoder position and clear integrator history.
			SetTargetToCurrentPosition();
			PIDITerm = 0.0f;
			last_vel_error = 0.0f;
			last_filtered_D = 0.0f;
			speedFilter.Reset();
			// The current loops' integrators go too. They hold whatever voltage was needed to sustain the
			// last move's current, which has nothing to do with this one, and starting a move by unwinding
			// a stale integrator is a torque transient at exactly the wrong moment.
			focIdIntegral = 0.0f;
			focIqIntegral = 0.0f;
			moveInstance->GetCurrentMotion(driverNumber, now, mParams); // Re-fetch after resync
		}

		// Open-loop FOC: drive field angle from step clock, ignore encoder.
		// Use M569 D5 (assistedOpen) to enable. Verifies commutation without a calibrated encoder.
		if (currentMode == ClosedLoopMode::assistedOpen)
		{
			// Deliberately derives the angle from the COMMANDED position and the configured C/L values,
			// not from the encoder or the alignment measurement: this mode exists to verify raw
			// commutation (phase order, pole-pair/CPR scaling) independent of encoder calibration, so it
			// must behave identically whether or not alignment has run. Consequence: if the measured
			// counts-per-electrical-rev disagrees with C/L (M569 D4 warns when it does), this mode
			// commutates at the wrong rate and the rotor tracks the commanded angle at that same wrong
			// ratio - which is itself a useful way to measure the true ratio.
			// mParams.position is in counts (1:1 with encoder counts). One electrical revolution = countsPerRev / polePairCount.
			const uint32_t countsPerElecRev = encoder->GetStepsPerRev() / polePairCount;
			uint16_t electricalAngle = 0;
			if (countsPerElecRev > 0)
			{
				int32_t remainder = (int32_t)llrintf(mParams.position) % (int32_t)countsPerElecRev;
				if (remainder < 0) { remainder += (int32_t)countsPerElecRev; }
				electricalAngle = (uint16_t)(((uint32_t)remainder * 4096u) / countsPerElecRev);
			}
			// Raised from an earlier fixed 0.3f cap: at low torque, the open-loop field can lose sync with
			// the rotor as the commanded electrical angle advances (rotor "slips" and snaps back, similar
			// to a stepper losing steps under too little holding torque for the commanded speed/inertia),
			// which looks like desynchronised/jumping motion rather than clean 1:1 tracking. 0.6 gives more
			// headroom to stay locked, while still being well under full authority and further bounded by
			// focMaxTorque and the voltage scale.
			const float assistedTorque = min<float>(0.6f, focMaxTorque) * GetFocVoltageScale();
			lastFocElectricalAngle = electricalAngle;
			lastFocTorqueMagnitude = assistedTorque;
			ApplyFocTorque(assistedTorque, electricalAngle);
			return assistedTorque;
		}

		const int32_t encoderCount = encoder->GetCurrentCount();
		const float targetPhysicalCount = (mParams.position * encoder->GetCountsPerStep()) * multiplier;
		currentPositionError = (targetPhysicalCount - (float)encoderCount) * encoder->GetStepsPerCount() * multiplier;
		speedFilter.ProcessReading((float)encoderCount * encoder->GetStepsPerCount(), now);

		const float timeDelta = (float)ticksSinceLastCall * (1.0f / (float)StepTimer::StepClockRate);

		// Outer position loop (P-controller → velocity setpoint)
		PIDJTerm = Kpp * currentPositionError;
		PIDVTerm = Kv * mParams.speed;
		PIDATerm = Ka * mParams.acceleration * (float)ticksSinceLastCall;
		const float vel_target = PIDJTerm + PIDVTerm + PIDATerm;

		// Inner velocity loop (PID)
		vel_measured = speedFilter.GetDerivative() * multiplier;
		const float vel_error = vel_target - vel_measured;
		PIDPTerm = Kp * vel_error;
		PIDITerm = constrain<float>(PIDITerm + (Ki * timeDelta * 0.5f * (vel_error + last_vel_error)), -PIDIlimit, PIDIlimit);
		const float rawD = (timeDelta > 0.0f) ? (vel_error - last_vel_error) / timeDelta : last_filtered_D;
		last_filtered_D += 0.1f * (rawD - last_filtered_D);
		PIDDTerm = Kd * last_filtered_D;

		PIDControlSignal = PIDPTerm + PIDITerm + PIDDTerm;

		// Compute the electrical angle from the encoder count, using the direction and scale MEASURED by
		// the alignment sweep (see focEncoderDirection/focCountsPerElecRev in ClosedLoop.h) rather than
		// assuming the electrical angle advances with increasing encoder count at the rate implied by
		// C and L. Because the alignment now settles the rotor's d-axis on electrical zero, angle
		// ComputeFocElectricalAngle(count) is the rotor's true electrical position, so ApplyTorque()'s
		// built-in +90 degrees lands the stator vector on the q-axis - maximum torque - at every count.
		const uint16_t electricalAngle = ComputeFocElectricalAngle(encoderCount);

		// Output authority, as a fraction of the bus. GetFocVoltageScale() (M569.1 N/O) converts
		// focMaxTorque from a fraction of the FULL, possibly much higher, supply voltage into a fraction of
		// a known ceiling - FocController applies whatever it is given directly as a duty cycle, with no
		// voltage scaling of its own.
		//
		// focEncoderDirection is applied to the demand below as well as to the commutation angle above, and
		// it must be: the angle correction makes a positive q command produce torque towards INCREASING
		// rotor electrical angle, but when the encoder counts the other way that is the direction of
		// DECREASING encoder count. Without this second factor the position loop's sign convention (which
		// is expressed in encoder counts, via multiplier) would be inverted relative to the torque it
		// actually gets, turning the loop into positive feedback. This is a hardware property and is
		// deliberately kept separate from the user-facing S0/S1 axis direction.
		const float focVoltageScale = GetFocVoltageScale();

		lastFocElectricalAngle = electricalAngle;

		// Resolve the measured phase currents onto the rotor axes BEFORE deciding the output - the current
		// loops below feed back from them, so a stale set would close the loop around the previous tick.
		MeasureFocDq(electricalAngle);

		const float maxOutput = focMaxTorque * focVoltageScale;

		// K clamps the DEMAND, not the current. The loops may command up to maxOutput regardless, and
		// maxOutput is a voltage: 0.25 of a 24V bus into a 0.45 ohm winding is over 13A. So the measured
		// current needs its own ceiling, above the demand limit but well inside the sense range, or a loop
		// that is misbehaving for any reason has nothing to stop it.
		const float measuredCurrentSq = (lastFocId * lastFocId) + (lastFocIq * lastFocIq);
		const float currentTripSq = (focMaxCurrent * 1.5f) * (focMaxCurrent * 1.5f);
		const bool currentWithinLimit = (measuredCurrentSq <= currentTripSq);

		if (FocCurrentModeActive() && FocMeasurementUsable() && currentWithinLimit)
		{
			// Current mode. The cascade's output stops being a voltage and becomes a q-axis current demand;
			// the PI loops below work out what voltage that takes. Full PID authority maps to focMaxCurrent
			// amps, the same role focMaxTorque plays for voltage mode.
			const float normalisedDemand = constrain<float>((PIDControlSignal * multiplier * (float)focEncoderDirection) / 256.0f, -1.0f, 1.0f);
			const float iqTarget = normalisedDemand * focMaxCurrent;
			lastFocIqTarget = iqTarget;

			// d is regulated to zero: current on the direct axis produces no torque, only heat. Driving it
			// to zero is most of the point of current mode - in voltage mode it is whatever the winding
			// impedance and the commutation error happen to leave there.
			const float idError = 0.0f - lastFocId;
			const float iqError = iqTarget - lastFocIq;

			focIdIntegral = constrain<float>(focIdIntegral + (focCurrentKi * idError * timeDelta), -maxOutput, maxOutput);
			focIqIntegral = constrain<float>(focIqIntegral + (focCurrentKi * iqError * timeDelta), -maxOutput, maxOutput);

			float vd = (focCurrentKp * idError) + focIdIntegral;
			float vq = (focCurrentKp * iqError) + focIqIntegral;

			// Circular limit, not per-axis clipping. Clipping d and q separately would shorten one component
			// more than the other and so ROTATE the applied vector away from where the loops asked for it,
			// which at saturation - exactly when the drive is working hardest - would swing torque onto the
			// direct axis. Scaling both by the same factor shortens the vector without turning it.
			const float mag = fastSqrtf((vd * vd) + (vq * vq));
			if (mag > maxOutput && mag > 0.0f)
			{
				const float scale = maxOutput / mag;
				vd *= scale;
				vq *= scale;
				// Anti-windup by back-calculation: pull the integrators back to what would have produced the
				// output we actually applied. Without this they keep accumulating against an error the
				// hardware cannot answer, and unwinding that takes as long as it took to build up.
				focIdIntegral *= scale;
				focIqIntegral *= scale;
			}

			focCurrentModeRunning = true;
			lastFocTorqueMagnitude = vq;
			ApplyFocDqVoltage(vd, vq, electricalAngle);
			RecordFocVoltages(vd, vq);
		}
		else
		{
			// Voltage mode, unchanged: the PID output IS the q-axis voltage, clamped rather than scaled so
			// that reducing W lowers the ceiling without also lowering the gain.
			//
			// Also the safe harbour when current mode is configured but the measurement has gone stale.
			// The integrators are cleared rather than held: whatever they contain was accumulated against
			// the readings that stopped being trustworthy, and on a saturation event that is precisely the
			// wound-up value that caused it. Resuming from it would re-enter the fault the moment the
			// measurement recovers.
			if (focCurrentModeRunning)
			{
				++focCurrentModeDropouts;
			}
			focIdIntegral = 0.0f;
			focIqIntegral = 0.0f;
			const float torqueMagnitude = constrain<float>((PIDControlSignal * multiplier * (float)focEncoderDirection) / 256.0f,
															-maxOutput, maxOutput);
			focCurrentModeRunning = false;
			lastFocIqTarget = 0.0f;
			lastFocTorqueMagnitude = torqueMagnitude;
			ApplyFocTorque(torqueMagnitude, electricalAngle);
			RecordFocVoltages(0.0f, torqueMagnitude);
		}

		last_vel_error = vel_error;
		return fabsf(PIDControlSignal) / 256.0f;
	}
#endif

	uint16_t commandedStepPhase;
	float currentFraction;

	if (inTorqueMode)
	{
		const uint32_t measuredStepPhase = encoder->GetCurrentPhasePosition();
#if 1
		// Limit the velocity by limiting the rate of rotation of the field (we could reduce the current too)
		if (torqueModeDirection)		// reverse movement
		{
			commandedStepPhase = (uint16_t)(((3 * 1024u) + measuredStepPhase) % 4096u);
			if (torqueModeMaxSpeed > 0.0 && speedFilter.GetDerivative() <= 0.0)
			{
				const uint32_t maxPhaseDecrement = (uint32_t)(torqueModeMaxSpeed * (1024 * ticksSinceLastCall)) % 4096;
				if ((desiredStepPhase - commandedStepPhase) % 4096u > maxPhaseDecrement)
				{
					commandedStepPhase = (uint16_t)((desiredStepPhase - maxPhaseDecrement) % 4096);
				}
			}
		}
		else						// forward movement
		{
			commandedStepPhase = (uint16_t)((measuredStepPhase + 1024u) % 4096u);
			if (torqueModeMaxSpeed > 0.0 && speedFilter.GetDerivative() >= 0.0)
			{
				const uint32_t maxPhaseIncrement = (uint32_t)(torqueModeMaxSpeed * (1024 * ticksSinceLastCall)) % 4096;
				if ((commandedStepPhase - desiredStepPhase) % 4096u > maxPhaseIncrement)
				{
					commandedStepPhase = (uint16_t)((desiredStepPhase + maxPhaseIncrement) % 4096);
				}
			}
		}

		currentFraction = torqueModeCommandedCurrentFraction;
#else
		// Limit the velocity by reducing the current if we are going too fast
		commandedStepPhase = (uint16_t)((((torqueModeDirection) ? (3 * 1024) : 1024) + measuredStepPhase) % 4096u);
		// For now we use a crude form of proportional control; we may need to improve it later.
		// If the speed is lower than the limit, increase the torque unless it is already at the requested torque.
		// If the speed is too high then reduce the torque.
		// It's likely that we will need to add a derivative term to prevent the speed oscillating.
		if (torqueModeMaxSpeed > 0.0)
		{
			const float rawVelocity = speedFilter.GetDerivative();
			const float speed = (torqueModeDirection) ? -rawVelocity : rawVelocity;
			const float speedErrorFraction = (speed - torqueModeMaxSpeed)/torqueModeMaxSpeed;
			const float torqueFactor = constrain<float>(VelocityLimitGainFactor * (1.0 - speedErrorFraction), 0.0, 1.0);
			currentFraction = torqueModeCommandedCurrentFraction * torqueFactor;
		}
		else
		{
			currentFraction = torqueModeCommandedCurrentFraction;
		}
#endif
	}
	else
{
		// Use a PID controller to calculate the required 'torque' - the control signal
		// We choose to use a PID control signal in the range -256 to +256. This is arbitrary.
		//PIDDTerm = constrain<float>(Kd * errorDerivativeFilter.GetDerivative() * StepTimer::StepClockRate, -256.0, 256.0);	// constrain D so that we can graph it more sensibly after a sudden step input
		
		if (currentMode == ClosedLoopMode::closed)
		{
						// For steppers, we will also use the cascaded controller.
			// The key is to perform the velocity loop calculations in ENCODER COUNTS, not steps.

			// 1. Outer Position Loop (P-controller) and Feedforward
			// The output is a corrective velocity in steps/tick.
			PIDJTerm = Kpp * currentPositionError;
			PIDVTerm = Kv * mParams.speed;
			PIDATerm = Ka * mParams.acceleration;

			// 2. Calculate Target Velocity for Inner Loop
			// Convert target velocity from steps/tick to counts/tick to match the units of the measured velocity.
			const float vel_target_steps = PIDJTerm + PIDVTerm;
			const float vel_target_counts = vel_target_steps / encoder->GetStepsPerCount();

			// 3. Inner Velocity Loop (PID-controller)
			// Recalculate measured velocity in counts/tick, NOT steps/tick.
			static DerivativeAveragingFilter<SpeedFilterSize> stepperSpeedFilter;
			stepperSpeedFilter.ProcessReading(encoder->GetCurrentCount(), now);
			vel_measured = stepperSpeedFilter.GetDerivative();
			const float vel_error = vel_target_counts - vel_measured;

			const float timeDelta = (float)ticksSinceLastCall * (1.0/(float)StepTimer::StepClockRate);						// get the time delta in seconds

			PIDPTerm = Kp * vel_error;
			PIDITerm = constrain<float>(PIDITerm + (Ki * timeDelta * 0.5f * (vel_error + last_vel_error)), -PIDIlimit, PIDIlimit);
			const float rawD = (timeDelta > 0.0f) ? (vel_error - last_vel_error) / timeDelta : 0.0f;
			PIDDTerm = Kd * (0.1f * rawD + 0.9f * last_filtered_D);
			last_filtered_D = (Kd > 0.0f) ? PIDDTerm / Kd : 0.0f;

			// 4. Final Control Signal
			PIDControlSignal = constrain<float>(PIDPTerm + PIDITerm + PIDDTerm + PIDATerm, -256.0, 256.0);
			last_vel_error = vel_error;
			
			// i.e. if we are moving in the positive direction, we must apply currents with a positive phase shift
			// The max abs value of phase shift we want is 1 full step i.e. 25%.
			// Given that PIDControlSignal is -256 .. 256 and phase is 0 .. 4095
			// and that 25% of 4096 = 1024, our max phase shift = 4 * PIDControlSignal

			// New algorithm: phase of motor current is always +/- 1 full step relative to current position, but motor current is adjusted according to the PID result
			// The following assumes that signed arithmetic is 2's complement
			const float PhaseFeedForwardFactor = 1000.0;
			const int16_t phaseFeedForward = lrintf(constrain<float>(speedFilter.GetDerivative() * ticksSinceLastCall * PhaseFeedForwardFactor, -256.0, 256.0));
			const uint32_t measuredStepPhase = encoder->GetCurrentPhasePosition();
			const uint16_t adjustedStepPhase = (uint16_t)((int16_t)measuredStepPhase + phaseFeedForward) % 4096u;
			commandedStepPhase = (((PIDControlSignal < 0.0) ? (3 * 1024) : 1024) + adjustedStepPhase) % 4096u;
			currentFraction = fabsf(PIDControlSignal) * (1.0/256.0);
		}
		else
		{
			// Driver is in assisted open loop mode
			PIDPTerm = constrain<float>(Kp * currentPositionError, -256.0, 256.0);
			PIDDTerm = constrain<float>(Kd * errorDerivativeFilter.GetDerivative() * StepTimer::StepClockRate, -256.0, 256.0);

			// In this mode the I term is not used and the A and V terms are independent of the loop time.
			constexpr float scalingFactor = 100.0;
			PIDVTerm = mParams.speed * Kv * scalingFactor;
			PIDATerm = mParams.acceleration * Ka * fsquare(scalingFactor);
			PIDControlSignal = min<float>(fabsf(PIDPTerm + PIDDTerm) + fabsf(PIDVTerm) + fabsf(PIDATerm), 256.0);

			const uint16_t stepPhase = (uint16_t)llrintf(mParams.position * 1024.0);		// we use llrintf so that we can guarantee to convert the float operand to integer. We only care about the lowest 12 bits.
			commandedStepPhase = (stepPhase + phaseOffset) % 4096u;
			currentFraction = holdCurrentFraction + (1.0 - holdCurrentFraction) * min<float>(PIDControlSignal * (1.0/256.0), 1.0);
		}
	}
	SetMotorPhase(commandedStepPhase, currentFraction);
	return currentFraction;
}

const char *_ecv_array ClosedLoop::GetModeText() const noexcept
{
	return (currentMode == ClosedLoopMode::closed) ? "closed loop"
			: (currentMode == ClosedLoopMode::assistedOpen) ? "assisted open loop"
				: "open loop";
}

void ClosedLoop::InstanceDiagnostics(size_t driver, const StringRef& reply) noexcept
{
	reply.printf("Closed loop driver %u mode: %s", driver, GetModeText());
	reply.catf(", pre-error threshold: %.2f, error threshold: %.2f", (double) errorThresholds[0], (double) errorThresholds[1]);
	if (motorType == EncoderType::bldc || motorType == EncoderType::stepperFoc || motorType == EncoderType::hybridStepperFoc)
	{
		reply.catf(", motor type %s (sensor: %s)", motorType.ToString(), GetEncoderType().ToString());
	}
	else
	{
		reply.catf(", encoder type %s", GetEncoderType().ToString());
	}
	if (encoder != nullptr)
	{
		if (!encoder->TakeReading())
		{
			reply.cat(", error reading encoder\n");
		}
		else
		{
			reply.catf(", position %" PRIi32 "\n", encoder->GetCurrentCount());
		}
		encoder->AppendDiagnostics(reply);
	}

	// The rest is only relevant if we are in closed loop mode
	if (currentMode != ClosedLoopMode::open)
	{
		reply.lcatf("Tuning mode: %#x, tuning error: %#x, collecting data: %s", tuning, tuningError, CollectingData() ? "yes" : "no");
		if (CollectingData())
		{
			reply.catf(" (filter: %#lx, mode: %u, rate: %u, movement: %u)", filterRequested, samplingMode, (unsigned int)(StepTimer::StepClockRate/dataCollectionIntervalTicks), movementRequested);
		}
#if SUPPORT_DCSERVO
		if (isDcServoMode)
		{
			reply.lcatf("DC PID snapshot: pos=%.1f enc=%" PRIi32 " err=%.2f P=%.1f I=%.1f D=%.1f out=%.1f mult=%.0f cmd=%d",
				(double)mParams.position, (encoder != nullptr) ? encoder->GetCurrentCount() : 0,
				(double)currentPositionError,
				(double)PIDPTerm, (double)PIDITerm, (double)PIDDTerm, (double)PIDControlSignal,
				(double)dcServoMultiplier, (int)hasMovementCommand);
			reply.lcatf("DM state=%u segs=%u curPos=%" PRIi32 " dcf=%.1f",
				moveInstance->GetDMState(driverNumber), moveInstance->CountSegments(driverNumber),
				moveInstance->GetCurrentMotorPosition(driverNumber), (double)moveInstance->GetDistanceCarriedForwards(driverNumber));
		}
#endif
#if SUPPORT_FOC
		if (motorType == EncoderType::bldc || motorType == EncoderType::stepperFoc || motorType == EncoderType::hybridStepperFoc)
		{
			// Kept deliberately terse: the whole diagnostics part shares one String<StringLength500>, and
			// overflowing it silently truncates whatever comes last - which is how the current-sense
			// lines went missing. Do not add verbose per-investigation debug here; put it behind a flag
			// or remove it once the question it answers has been settled. The position cross-check that
			// used to live here was exactly that, and its bug is long fixed.
			reply.lcatf("FOC: align=%d move=%d poles=%u maxT=%.2f mult=%.0f, counts/elecRev=%" PRIu32 " (%s) dir%+d",
				(int)focAlignmentDone, (int)hasMovementCommand, (unsigned)polePairCount, (double)focMaxTorque,
				(double)focMultiplier, GetFocCountsPerElecRev(),
				(focCountsPerElecRev != 0) ? "measured" : "C/L", (int)focEncoderDirection);
			{
				const float electricalAngleDegrees = ((float)lastFocElectricalAngle * 360.0f) / 4096.0f;
				reply.lcatf("FOC last cmd: angle=%u (%.0fdeg) torque=%.4f enc=%" PRIi32 " fault=%s",
					(unsigned)lastFocElectricalAngle, (double)electricalAngleDegrees, (double)lastFocTorqueMagnitude,
					(encoder != nullptr) ? encoder->GetCurrentCount() : 0, (lastFocDriverFault) ? "YES" : "no");
			}
			if (focController != nullptr)
			{
				reply.lcatf("FOC duties: U=%.3f V=%.3f W=%.3f",
					(double)focController->lastDutyU, (double)focController->lastDutyV, (double)focController->lastDutyW);
			}
			else
			{
				reply.lcat("FOC: focController is NULL");
			}
		}
#endif
	}

	//DEBUG
	//reply.catf(", event status 0x%08" PRIx32 ", TCC2 CTRLA 0x%08" PRIx32 ", TCC2 EVCTRL 0x%08" PRIx32, EVSYS->CHSTATUS.reg, QuadratureTcc->CTRLA.reg, QuadratureTcc->EVCTRL.reg);
}

// Gate-driver and current-sense hardware state.
//
// Reported as its own M122 part rather than appended to the closed-loop part above, because each part
// shares a single String<StringLength500> and the closed-loop part alone already fills most of it. When
// it overflows the tail is silently dropped - which is exactly how these lines went missing the first
// time, with no indication that anything had been truncated.
void ClosedLoop::InstanceDriverDiagnostics(size_t driver, const StringRef& reply) noexcept
{
#if SUPPORT_DRV8316_SPI
	if (drv8316 != nullptr)
	{
		if (drv8316->IsPresent())
		{
			reply.lcatf("DRV8316: IC=0x%02x S1=0x%02x S2=0x%02x, config %s,",
				drv8316->GetIcStatus(), drv8316->GetStatus1(), drv8316->GetStatus2(),
				drv8316->IsConfigVerified() ? "verified" : "NOT VERIFIED");
			drv8316->AppendFaultDescription(reply);
		}
		else
		{
			reply.lcat("DRV8316: SPI no response (check wiring, nSLEEP)");
		}
		FocCurrentSense::AppendDiagnostics(reply);

		// The rotor-frame view, reported here rather than with the other FOC state in part 7 because it is
		// derived from the current sense and because part 7 has already overflowed its 500-char reply once.
		//
		// What to look for while the motor is holding or moving under load: |Id| should be small next to
		// |Iq|. Id is the component of stator current that produces no torque, so a large one means the
		// commutation angle is wrong - current being pushed into the rotor's direct axis, heating the motor
		// for nothing. Iq should share its sign with Vq. Both hold in voltage mode, where Vd is zero by
		// construction; they are the precondition for closing current loops around this frame.
		reply.lcatf("FOC dq: Id=%.3fA Iq=%.3fA Vd=%.2f Vq=%.2f%s",
			(double)lastFocId, (double)lastFocIq, (double)lastFocVd, (double)lastFocVq,
			(focSupplyVoltage > 0.0f) ? "V" : " (bus fraction, M569.1 N unset)");
		if (focCurrentKp > 0.0f)
		{
			// Says whether the loops are actually running, not just configured - the preconditions in
			// FocCurrentModeActive() are easy to miss and failing them looks like nothing happening.
			reply.lcatf("FOC current mode: %s, Kp=%.4f Ki=%.1f maxI=%.2fA, IqTarget=%.3fA, integ d=%.3f q=%.3f, dropouts=%" PRIu32,
				(focCurrentModeRunning) ? "RUNNING" : ((FocCurrentModeActive()) ? "FELL BACK to voltage mode (stale measurement)" : "configured but INACTIVE"),
				(double)focCurrentKp, (double)focCurrentKi, (double)focMaxCurrent,
				(double)lastFocIqTarget, (double)focIdIntegral, (double)focIqIntegral, focCurrentModeDropouts);
		}
	}
	else
	{
		reply.lcat("DRV8316: not configured (M569.1 T5 not yet sent)");
	}
#else
	(void)driver; (void)reply;
#endif
}

/*static*/ void ClosedLoop::Diagnostics(const StringRef& reply) noexcept
{
	for (size_t i = 0; i < NumDrivers; ++i)
	{
		moveInstance->ClosedLoopDiagnostics(i, reply);
	}
}

/*static*/ void ClosedLoop::DriverDiagnostics(const StringRef& reply) noexcept
{
	for (size_t i = 0; i < NumDrivers; ++i)
	{
		moveInstance->ClosedLoopDriverDiagnostics(i, reply);
	}
}

StandardDriverStatus ClosedLoop::ReadLiveStatus() const noexcept
{
	StandardDriverStatus result;
	result.all = 0;
	result.closedLoopPositionNotMaintained = stall;
	result.closedLoopPositionWarning = preStall;
	result.closedLoopNotTuned = ((tuningError & encoder->MinimalTuningNeeded()) != 0);
	result.closedLoopTuningError = ((tuningError & TuningError::AnyTuningFailure) != 0);
#if SUPPORT_DRV8316_SPI
	if (drv8316 != nullptr && drv8316->IsPresent() && drv8316->HasFault())
	{
		result.closedLoopPositionNotMaintained = true;	// gate driver fault
		if (drv8316->HasOtw()) { result.otpw = true; }	// over-temperature warning
		if (drv8316->HasOcp()) { result.s2ga = true; }	// use short-to-ground as proxy for OCP
	}
#endif
	return result;
}

void ClosedLoop::ResetError() noexcept
{
# if SINGLE_DRIVER
	if (encoder != nullptr)
	{
		TaskCriticalSectionLocker lock;

		// Set the target position to the current position
		const bool ok = encoder->TakeReading();
		(void)ok;		//TODO handle error
		errorDerivativeFilter.Reset();
		speedFilter.Reset();
		SetTargetToCurrentPosition();
		inTorqueMode = false;

		// Explicitly clear any latched stall condition. This is the primary mechanism for recovering from a fault.
		stall = false;
		preStall = false;
	}
# else
#  error Multi driver code not implemented
# endif
}

// This is called before the driver mode is changed. Return true if success. Always succeeds if we are disabling closed loop.
bool ClosedLoop::SetClosedLoopEnabled(ClosedLoopMode mode, const StringRef &reply) noexcept
{
	// Trying to enable closed loop
	if (mode != ClosedLoopMode::open)
	{
		if (encoder == nullptr)
		{
			reply.copy("No encoder specified for closed loop drive mode");
			return false;
		}

		if (currentMode == ClosedLoopMode::open)
		{
#if SUPPORT_TMC51xx
			// Switching from open to closed loop mode, so set the motor phase to match the current microstep position
			delay(10);													// delay long enough for the TMC driver to have read the microstep counter since the end of the last movement
			const uint16_t initialStepPhase = SmartDrivers::GetMicrostepPosition(driverNumber) * 4;	// get the current coil A microstep position as 0..4095

			// Temporarily calibrate the encoder zero position
			// We assume that the motor is at the position given by its microstep counter. This may not be true e.g. if it has a brake that has not been disengaged.
			const bool ok = encoder->TakeReading();
			if (!ok)
			{
				reply.copy("Error reading encoder");
				return false;
			}
			desiredStepPhase = initialStepPhase;						// set this to be picked up later in DriverSwitchedToClosedLoop
#endif
		}

		if (encoder->UsesBasicTuning() && (tuningError & TuningError::NeedsBasicTuning) != 0)
		{
			encoder->SetTuningBackwards(false);
			encoder->SetKnownPhaseAtCurrentCount(desiredStepPhase);
		}

		PIDITerm = 0.0;
		errorDerivativeFilter.Reset();
		speedFilter.Reset();

		// Set the target position to the current position (ResetError() calls SetTargetToCurrentPosition()
		// internally, among other things). For FOC motors entering closed loop, both are deferred until
		// after the alignment dwell below zeroes the encoder at the rotor's true electrical-zero rest
		// position: calling either here would seed targetMotorSteps from the pre-alignment (effectively
		// meaningless) encoder reading, and that stale value could then be observed by the periodic
		// control-loop task if it samples state in between the alignment dwell zeroing the encoder and
		// re-syncing the target below.
#if SUPPORT_FOC
		if (!(mode == ClosedLoopMode::closed
				&& focController != nullptr
				&& (motorType == EncoderType::bldc
					|| motorType == EncoderType::stepperFoc
					|| motorType == EncoderType::hybridStepperFoc)))
#endif
		{
			ResetError();												// this calls ReadState again, sets up targetMotorSteps, and clears any latched stall
		}

		moveInstance->ResetPhaseStepMonitoringVariables();				// to avoid getting stupid values
		moveInstance->ResetPhaseStepControlLoopCallTime();				// to avoid huge integral term windup

#if SUPPORT_FOC
		// FOC closed-loop: run the alignment dwell now, before any move is queued.
		// Hold the field at angle 0 for 200ms so the rotor settles to a known position,
		// then enable the encoder (zeroed at that position) and mark alignment done.
		// This runs on the calling task (command processor), not the control loop task.
		if (mode == ClosedLoopMode::closed
			&& focController != nullptr
			&& (motorType == EncoderType::bldc
				|| motorType == EncoderType::stepperFoc
				|| motorType == EncoderType::hybridStepperFoc))
		{
#if SUPPORT_DRV8316_SPI
			if (drv8316 != nullptr) { drv8316->ClearFaults(); }

			// Establish the zero-current point before anything is driven. This has to happen with the
			// output stage idle, and the moment before the alignment sweep starts is the only point in
			// the sequence where that is guaranteed. Reported but not fatal: a bad calibration means the
			// current readings are wrong, not that commutation is.
			(void)FocCurrentSense::CalibrateZeroOffset(reply);

			// Hand the ADC to the PWM carrier now rather than after alignment. The sweep below measures the
			// current-sense frame against the drive frame, and that measurement wants conversions taken at
			// the centre of the low-side conduction window like every other one - a polled read would sample
			// switching noise at an arbitrary point in the carrier. The carrier is already running (the TCC
			// is configured in FocController::Init), so there is nothing to wait for.
			FocCurrentSense::StartSynchronisedSampling();
#endif
			// NB: SetClosedLoopEnabled() runs synchronously on the command-processor task while the main
			// board waits for a CAN reply with a fixed ~1000ms timeout (CanInterface::UsualResponseTimeout
			// on the main board side) - the WHOLE alignment sequence below must fit comfortably under that
			// budget or M569 D4/M569.1 will time out and the main board will report "CAN response timeout".
			// Keep the total well under 900ms.
			//
			// Alignment torque is capped by focMaxTorque (M569.1 W), same ceiling as running torque, so
			// a user who has reduced W gets a correspondingly reduced alignment pull too, rather than a
			// fixed 0.5 regardless of W. Also scaled by GetFocVoltageScale() (M569.1 N/O) so that, once
			// configured, the alignment pull respects the same real-volts ceiling as running torque
			// instead of being an unscaled fraction of the full, possibly much higher, supply voltage.
			float alignTorque = min<float>(0.5f, focMaxTorque) * GetFocVoltageScale();


			// Align at 3*pi/2, NOT at 0. ApplyTorque() issues a pure q-axis command (d = 0), and the
			// inverse Park transform that implements it - alpha = -q*sin(theta), beta = q*cos(theta), see
			// FocController::InversePark() - therefore places the stator vector 90 electrical degrees
			// AHEAD of the angle passed in. That is correct and expected for a torque command, but it
			// means an alignment pull commanded at angle 0 parks the rotor's d-axis at +90 degrees. The
			// runtime commutation formula then adds the same +90 on top, so the vector applied at the
			// alignment origin lands exactly ON the rotor d-axis: zero torque for ANY commanded
			// magnitude, and torque proportional to sin(angle error) instead of cos(angle error) either
			// side of it. The practical result is a zero-torque magnetic trap at the alignment origin
			// that is STABLE for one sign of torque - the harder the loop pushes, the tighter it clamps -
			// which is exactly the "cannot move in one direction at all, hits an invisible wall" failure.
			// Sweeping to 3*pi/2 cancels the built-in +90 so that encoder count 0 is genuinely electrical
			// zero. This is the same reason SimpleFOC's alignSensor() uses setPhaseVoltage(v, 0, _3PI_2)
			// rather than angle 0.
			constexpr uint16_t alignTargetAngle = 3072u;

#if SUPPORT_DRV8316_SPI
			// Back the alignTorque ceiling computed above off to whatever actually produces a sensible
			// current on THIS motor. Holding alignTargetAngle throughout, so this doubles as a pre-settle.
			//
			// 0.5 is half the bus - 12V into a winding whose resistance is well under an ohm. Measured on
			// the DRV8316 EVM it drove both current-sense amplifiers hard into their rails (+7.2A at ADC
			// full scale, -6.0A at zero) for 99% of the sweep, with Kirchhoff sums of several amps, i.e.
			// arithmetically impossible readings. That is bad on its own account - it is well over the
			// motor's continuous rating, even if it stays under the driver's 16A overcurrent trip - and it
			// also makes the sweep useless as a calibration, because a clipped waveform carries no reliable
			// angle information.
			//
			// So ramp up from a small pull while watching the current, and stop at a target that leaves the
			// amplifiers plenty of linear headroom. This can only ever REDUCE the torque below the value
			// computed above, so it cannot make alignment weaker than the ceiling the user configured via
			// W, and the floor keeps enough pull to drag the rotor past detents.
			if (FocCurrentSense::IsInitialised())
			{
				constexpr float AlignTargetCurrent = 2.5f;			// amps, ~40% of the +-6A sense range
				constexpr float AlignMinTorque = 0.02f;				// never ramp below this
				// x2ms, so 50ms worst case on top of the ~700ms sequence, against the ~900ms CAN reply
				// budget noted above. In practice it exits after a few steps: the target current is
				// reached at a small fraction of the ceiling, which is the whole point.
				constexpr unsigned int RampSteps = 25;

				float found = alignTorque;
				for (unsigned int step = 1; step <= RampSteps; ++step)
				{
					const float trial = (alignTorque * (float)step) / (float)RampSteps;
					focController->ApplyTorque(trial, alignTargetAngle);
					delay(2);
					if (!FocCurrentSense::RefreshAllPhases(2000)) { continue; }
					float ia, ib, ic;
					FocCurrentSense::GetPhaseCurrents(ia, ib, ic);
					// Peak phase current, which is what saturates an amplifier - not the vector magnitude.
					const float peak = max<float>(fabsf(ia), max<float>(fabsf(ib), fabsf(ic)));
					if (peak >= AlignTargetCurrent)
					{
						found = trial;
						break;
					}
				}
				// Floor applied only when it does not exceed the ceiling: constrain() with lo > hi returns
				// lo, which on a rig configured with a very small W would have RAISED the torque above the
				// limit the user asked for - the opposite of this block's purpose.
				alignTorque = (AlignMinTorque < alignTorque) ? max<float>(found, AlignMinTorque) : alignTorque;
				focAlignTorqueUsed = alignTorque;
			}
#endif

			// Three phases, ~700ms total (unchanged from the previous 500+200 budget):
			//   1. settle at alignTargetAngle                      -> record the count
			//   2. sweep one full electrical revolution
			//   3. settle at alignTargetAngle again                -> record the count
			// The difference between the two recorded counts is a direct, SIGNED measurement of one
			// electrical revolution in encoder counts: its sign is the encoder-to-electrical direction
			// (which nothing in this firmware previously determined - it was assumed +1) and its
			// magnitude is countsPerElecRev as the motor actually behaves, rather than as C and L imply.
			//
			// The sweep itself is retained from the previous implementation for the reason documented
			// there: a held fixed-angle pull is open-loop AND static, so a detent between the rotor's
			// unknown start position and the target can stop it short and get it zeroed in the wrong
			// place (this showed up as repeated 40-90 degree shortfalls). A continuously-advancing field
			// drags the rotor past every detent at least once regardless of where it started. Phase 1 is
			// new and exists so that BOTH endpoints of the measurement are settled at the same commanded
			// angle - without it the start point is contaminated by the rotor snapping in from wherever
			// it happened to be powered up, which is what made the old preAlignCount-based check able to
			// report only a vague "plausible amount of motion" rather than a usable calibration.
			constexpr StepTimer::Ticks preSettleTicks     = (StepTimer::StepClockRate * 12) / 100;	// 120 ms
			constexpr StepTimer::Ticks sweepDurationTicks = (StepTimer::StepClockRate * 40) / 100;	// 400 ms
			constexpr StepTimer::Ticks holdDurationTicks  = (StepTimer::StepClockRate * 18) / 100;	// 180 ms

			// Phase 1: settle at the start angle.
			{
				const StepTimer::Ticks phaseStart = StepTimer::GetTimerTicks();
				while ((StepTimer::Ticks)(StepTimer::GetTimerTicks() - phaseStart) < preSettleTicks)
				{
					focController->ApplyTorque(alignTorque, alignTargetAngle);
					delay(1);
				}
			}
			encoder->TakeReading();
			const int32_t sweepStartCount = encoder->GetCurrentCount();

			// Phase 2: linear ramp through one full electrical revolution, ending back at alignTargetAngle.
			//
			// This sweep doubles as the calibration of the current-sense frame against the drive frame,
			// which is why the current is sampled here and nowhere else. Every other opportunity is
			// confounded: during normal running the commutation angle comes from the encoder (so a wrong
			// counts-per-electrical-rev, or a wrong encoder direction, is indistinguishable from a wrong
			// sense mapping), and the torque command changes between control ticks (so the three phases,
			// which are sampled one per tick, are captured at three different current amplitudes and the
			// reconstructed vector is meaningless). Here the commanded angle is known exactly, owes nothing
			// to the encoder, and the torque is constant - so the only thing left that can rotate the
			// measured current vector is the sense-to-drive mapping itself.
			//
			// ApplyTorque() places the voltage vector 90 degrees ahead of the commanded angle, and at this
			// sweep rate (one electrical revolution in 400ms) back-EMF and reactance are both negligible, so
			// the current vector should sit at commandedAngle + 90 too. Two hypotheses are accumulated: the
			// sense frame agreeing with the drive frame, and it being mirrored (the signature of the phase
			// leads or the ISEN inputs being in opposite rotational order, which a working motor does not
			// reveal because the alignment sweep simply measures the reversed direction and compensates).
			// Whichever has the tighter spread wins.
			{
				const StepTimer::Ticks phaseStart = StepTimer::GetTimerTicks();
#if SUPPORT_DRV8316_SPI
				float alignedSin = 0.0f, alignedCos = 0.0f, mirroredSin = 0.0f, mirroredCos = 0.0f, weightSum = 0.0f;
				unsigned int frameSamples = 0, clippedSamples = 0;
#endif
				while (true)
				{
					const StepTimer::Ticks elapsed = (StepTimer::Ticks)(StepTimer::GetTimerTicks() - phaseStart);
					if (elapsed >= sweepDurationTicks) { break; }
					const uint32_t sweepProgress = ((uint32_t)elapsed * 4096u) / sweepDurationTicks;
					const uint16_t commandedAngle = (uint16_t)((sweepProgress + alignTargetAngle) & 4095u);
					focController->ApplyTorque(alignTorque, commandedAngle);
#if SUPPORT_DRV8316_SPI
					// A full set captured within a few hundred microseconds, rather than the one-phase-per-
					// tick the control loop settles for. 2ms is generous against the ~150us it should take.
					if (FocCurrentSense::RefreshAllPhases(2000))
					{
						float ia, ib, ic;
						FocCurrentSense::GetPhaseCurrents(ia, ib, ic);
						constexpr float OneOverSqrt3 = 0.5773502692f;
						const float ialpha = (2.0f * ia - ib - ic) * (1.0f / 3.0f);
						const float ibeta  = (ib - ic) * OneOverSqrt3;
						const float mag = fastSqrtf((ialpha * ialpha) + (ibeta * ibeta));
						// A clipped sample carries no usable angle: once an amplifier rails, the waveform
						// is flat-topped and the reconstructed vector swings towards the unclipped phases.
						// Kirchhoff is the cheapest detector - three real phase currents sum to zero, so a
						// large sum means at least one reading is not a real current. Counted rather than
						// silently dropped, because a sweep that is mostly clipped has not measured
						// anything and must say so.
						if (fabsf(ia + ib + ic) > 0.5f)
						{
							++clippedSamples;
						}
						else if (mag > 0.05f)				// ignore samples that are all offset and noise
						{
							const float phiI = atan2f(ibeta, ialpha);
							const float phiV = (((float)commandedAngle * TwoPi) / 4096.0f) + (Pi * 0.5f);
							// Weighting by magnitude makes this a proper vector average: strong samples,
							// where the angle is well determined, count for more than weak ones.
							alignedSin  += mag * sinf(phiI - phiV);
							alignedCos  += mag * cosf(phiI - phiV);
							mirroredSin += mag * sinf(phiI + phiV);
							mirroredCos += mag * cosf(phiI + phiV);
							weightSum += mag;
							++frameSamples;
						}
					}
#endif
					delay(1);
				}
#if SUPPORT_DRV8316_SPI
				focSenseFrameClipped = clippedSamples;
				if (weightSum > 0.0f && frameSamples >= 50 && clippedSamples * 4 < frameSamples)
				{
					const float rAligned  = fastSqrtf((alignedSin * alignedSin) + (alignedCos * alignedCos)) / weightSum;
					const float rMirrored = fastSqrtf((mirroredSin * mirroredSin) + (mirroredCos * mirroredCos)) / weightSum;
					const bool mirrored = (rMirrored > rAligned);
					focSenseFrameDirection = (mirrored) ? -1 : 1;
					focSenseFrameConfidence = (mirrored) ? rMirrored : rAligned;
					focSenseFrameOffsetDeg = (mirrored)
							? (atan2f(mirroredSin, mirroredCos) * RadiansToDegrees)
							: (atan2f(alignedSin, alignedCos) * RadiansToDegrees);
					focSenseFrameSamples = frameSamples;

					// Snap to the 60-degree grid before using it. Only multiples of 60 are physically
					// reachable - see SenseFrameCorrection - so anything else in the measurement is the
					// winding's load angle plus noise, and rotating that away would be calibrating out
					// real physics. On the DRV8316 EVM the raw figure came out at +66 degrees, which is
					// the 60 of an A/C channel swap with inverted amplifier polarity, plus 6 degrees of
					// genuine lag.
					const float snapped = lrintf(focSenseFrameOffsetDeg / 60.0f) * 60.0f;
					focSenseFrameResidualDeg = focSenseFrameOffsetDeg - snapped;
					focSenseCorrection.mirrored = mirrored;
					focSenseCorrection.cosOffset = cosf(snapped * DegreesToRadians);
					focSenseCorrection.sinOffset = sinf(snapped * DegreesToRadians);
				}
				else
				{
					// Leave the correction at identity: an unmeasured frame must not silently rotate the
					// currents by whatever the last run happened to find.
					focSenseFrameDirection = 0;				// not enough current to say anything
					focSenseFrameConfidence = 0.0f;
					focSenseFrameOffsetDeg = 0.0f;
					focSenseFrameResidualDeg = 0.0f;
					focSenseFrameSamples = frameSamples;
					focSenseCorrection = FocController::SenseFrameCorrection();
				}
#endif
			}

			// Phase 3: settle at the same angle we measured from, exactly one electrical revolution later.
			{
				const StepTimer::Ticks phaseStart = StepTimer::GetTimerTicks();
				while ((StepTimer::Ticks)(StepTimer::GetTimerTicks() - phaseStart) < holdDurationTicks)
				{
					focController->ApplyTorque(alignTorque, alignTargetAngle);
					delay(1);
				}
			}
			encoder->TakeReading();
			const int32_t measuredElecRevCounts = encoder->GetCurrentCount() - sweepStartCount;	// signed: one electrical revolution

			// Adopt the measurement, but only if it is in the same ballpark as the configured CPR/pole
			// pair count. A blocked, hand-held or unpowered shaft would otherwise poison commutation
			// outright, which is worse than the assumption we are replacing. Outside the band we keep the
			// derived value and raise a tuning error, which stops InstanceControlLoop() from driving the
			// motor at all until it is cleared. The band is deliberately wide (0.5x to 2x): its job is to
			// reject a failed measurement, not to police a genuine CPR/pole-pair misconfiguration, which
			// is reported below instead so that the drive still runs on the measured value.
			const uint32_t derivedCountsPerElecRev = (polePairCount != 0)
					? (uint32_t)((encoder->GetCountsPerStep() * (float)encoder->GetStepsPerRev()) / (float)polePairCount)
					: 0;
			const uint32_t measuredMagnitude = (uint32_t)labs(measuredElecRevCounts);
			if (derivedCountsPerElecRev != 0
				&& measuredMagnitude >= derivedCountsPerElecRev / 2
				&& measuredMagnitude <= derivedCountsPerElecRev * 2)
			{
				focEncoderDirection = (measuredElecRevCounts < 0) ? -1 : 1;
				focCountsPerElecRev = measuredMagnitude;
				tuningError &= ~(TuningError::TooLittleMotion | TuningError::TooMuchMotion);
			}
			else
			{
				focEncoderDirection = 1;
				focCountsPerElecRev = 0;						// fall back to the derived value
				tuningError = (tuningError & ~(TuningError::TooLittleMotion | TuningError::TooMuchMotion))
							| ((measuredMagnitude < derivedCountsPerElecRev) ? TuningError::TooLittleMotion : TuningError::TooMuchMotion);
			}

			// Zero the encoder at the settled position and re-synchronise the target atomically.
			// This must all happen inside one critical section: the periodic control-loop task
			// reads/writes this same state (encoder count, target) on every ~80us tick regardless
			// of currentMode, so without the lock it can observe a torn combination of a freshly
			// zeroed encoder paired with a stale, not-yet-resynchronised target - producing a large
			// phantom position error that the PID then reacts to at full authority.
			{
				TaskCriticalSectionLocker lock;
				encoder->TakeReading();		// refresh currentCount one last time before we zero it
				encoder->Enable();				// zeroes the hardware counter and the cached currentCount at the settled position
				encoder->TakeReading();		// flush stale currentCount; Enable() zeroed hardware but not the cached count
				SetTargetToCurrentPosition();
				PIDITerm = 0.0f;
				errorDerivativeFilter.Reset();
				speedFilter.Reset();
				inTorqueMode = false;
				stall = false;					// explicitly clear any latched stall condition, same as ResetError()
				preStall = false;
				focAlignmentDone = true;
			}

			// Report the measured calibration against the configured one. A large disagreement here does
			// NOT stop the drive (we run on the measured value, which is what the motor actually does),
			// but it means C and/or L do not describe this motor/encoder combination, and every OTHER
			// consumer of that configuration - assistedOpen commutation, steps/mm, the plausibility band
			// above - is still working from the wrong number, so it is worth surfacing loudly.
			{
				const int32_t derivedForReport = (int32_t)derivedCountsPerElecRev;
				reply.lcatf("Alignment: 1 electrical rev = %" PRIi32 " counts measured (direction %s), %" PRIi32 " counts from C/L",
					(int32_t)measuredElecRevCounts, (focEncoderDirection < 0) ? "reversed" : "forward", derivedForReport);
				if (focCountsPerElecRev == 0)
				{
					reply.cat(" - measurement rejected, using C/L value; check the motor is powered and the shaft is free");
				}
				else if (derivedForReport != 0
						&& (int32_t)labs((int32_t)focCountsPerElecRev - derivedForReport) * 10 > derivedForReport)
				{
					reply.cat(" - WARNING: >10% disagreement, check M569.1 C (encoder CPR) and L (pole pairs)");
				}
			}

#if SUPPORT_DRV8316_SPI
			// Report the sense-frame orientation measured during the sweep above. A mirrored frame does not
			// stop the motor working - commutation never consults the current sense - but it inverts every
			// d/q quantity derived from it, so it must be resolved before any current loop is closed.
			if (focAlignTorqueUsed > 0.0f)
			{
				reply.lcatf("Alignment torque: %.3f of bus (limited by current ramp)", (double)focAlignTorqueUsed);
			}
			if (focSenseFrameDirection != 0)
			{
				reply.lcatf("Current sense frame: %s drive frame (offset %+.0f deg, confidence %.2f over %u samples)",
					(focSenseFrameDirection < 0) ? "MIRRORED vs" : "agrees with",
					(double)focSenseFrameOffsetDeg, (double)focSenseFrameConfidence, (unsigned)focSenseFrameSamples);
				if (focSenseFrameConfidence < 0.5f)
				{
					reply.cat(" - low confidence, treat as unmeasured");
				}
				// The frame line always describes the HARDWARE, measured from raw readings, so it keeps
				// reporting MIRRORED after the correction is in place - that is deliberate, it stays a live
				// check on the wiring. This second line says what was done about it.
				reply.lcatf("Current sense corrected by %s%.0f deg, residual %+.1f deg (load angle, not corrected)",
					(focSenseCorrection.mirrored) ? "mirror + " : "",
					(double)(lrintf(focSenseFrameOffsetDeg / 60.0f) * 60.0f), (double)focSenseFrameResidualDeg);
				if (fabsf(focSenseFrameResidualDeg) > 20.0f)
				{
					reply.cat(" - WARNING: residual too large to be a load angle, the 60-degree snap is suspect");
				}
			}
			else if (focSenseFrameClipped * 4 >= focSenseFrameSamples && focSenseFrameClipped != 0)
			{
				// Distinguish "no signal" from "too much signal": they need opposite responses, and a
				// saturated sweep previously reported a confident, precise and completely wrong answer.
				reply.lcatf("Current sense frame: NOT measured - %u of %u sweep samples saturated the current sense; "
							"reduce M569.1 W or set a voltage limit with M569.1 N/O",
					(unsigned)focSenseFrameClipped, (unsigned)(focSenseFrameClipped + focSenseFrameSamples));
			}
			else
			{
				reply.lcatf("Current sense frame: not measured (%u usable samples during the sweep)", (unsigned)focSenseFrameSamples);
			}
#endif

			// Alignment is complete. Note there is deliberately no q-axis offset search here: sweeping to
			// 3*pi/2 above puts the rotor d-axis on electrical zero, so a positive torqueMagnitude is a
			// true q-axis command at every encoder count by construction. A fixed-test-offset ramp and a
			// live-torque-response search over 0/90/180/270 both used to live at this point; both were
			// scaffolding for the alignment-reference and commutation-direction bugs, and the evidence
			// that appeared to justify them was an artefact of those same bugs.
		}
#endif
	}

	// If we are disabling closed loop mode, we should ideally send steps to get the microstep counter to match the current phase here
#if SUPPORT_FOC
	// Discard the measured commutation calibration along with the alignment when dropping to open loop:
	// the usual reason we end up here is M569.1 about to delete and rebuild the encoder, after which a
	// stale counts-per-electrical-rev measured against the previous CPR would be actively wrong.
	if (mode == ClosedLoopMode::open)
	{
		focAlignmentDone = false;
		focEncoderDirection = 1;
		focCountsPerElecRev = 0;
	}
#endif
	currentMode = mode;

	return true;
}

// This is called just after the driver has switched into closed loop mode (it may have been in closed loop mode already)
void ClosedLoop::DriverSwitchedToClosedLoop() noexcept
{
	delay(3);														// allow time for the switch to complete and a few control loop iterations to be done
	const uint16_t currentPhasePosition = (uint16_t)encoder->GetCurrentPhasePosition();
	if (currentMode == ClosedLoopMode::assistedOpen)
	{
		const uint16_t stepPhase = (uint16_t)llrintf(mParams.position * 1024.0);
		phaseOffset = (currentPhasePosition - stepPhase) & 4095;
	}
	desiredStepPhase = currentPhasePosition;
#if SUPPORT_TMC51xx
	SetMotorPhase(currentPhasePosition, SmartDrivers::GetStandstillCurrentPercent(driverNumber) * 0.01);	// set the motor currents to match the initial position using the open loop standstill current
#endif
	PIDITerm = 0.0;													// clear the integral term accumulator
	errorDerivativeFilter.Reset();
	speedFilter.Reset();
	moveInstance->ResetPhaseStepMonitoringVariables();				// the first loop iteration will have recorded a higher than normal loop call interval, so start again
}

// If we are in closed loop modify the driver status appropriately
StandardDriverStatus ClosedLoop::ModifyDriverStatus(StandardDriverStatus originalStatus) const noexcept
{
	if (currentMode != ClosedLoopMode::open)
	{
		originalStatus.stall = 0;										// ignore stall detection in open loop mode
		originalStatus.standstill = 0;									// ignore standstill detection in closed loop mode
		originalStatus.closedLoopNotTuned = ((tuningError & encoder->MinimalTuningNeeded()) != 0);
		originalStatus.closedLoopIllegalMove = 0;						//TODO implement this or remove it
		originalStatus.closedLoopTuningError = ((tuningError & TuningError::AnyTuningFailure) != 0);
	}

	if (!originalStatus.closedLoopNotTuned)
	{
		// Report position warnings and errors even in open loop mode, if tuning has been done
		originalStatus.closedLoopPositionWarning = preStall;
		originalStatus.closedLoopPositionNotMaintained = stall;
	}

	return originalStatus;
}

// Get the current fraction and position error statistics
void ClosedLoop::GetStatistics(ClosedLoopStatus& stat) noexcept
{
	TaskCriticalSectionLocker lock;

	if (periodNumSamples == 0)
	{
		stat.averageCurrentFraction = stat.maxCurrentFraction = stat.rmsPositionError = stat.maxAbsPositionError = 0.0;
	}
	else
	{
		stat.averageCurrentFraction = (float16_t)(periodSumOfCurrentFractions/periodNumSamples);
		stat.maxCurrentFraction = (float16_t)periodMaxCurrentFraction;
		stat.rmsPositionError = (float16_t)fastSqrtf(periodSumOfPositionErrorSquares/periodNumSamples);
		stat.maxAbsPositionError = (float16_t)periodMaxAbsPositionError;

		// Clear them out ready for the next period
		periodNumSamples = 0;
		periodSumOfCurrentFractions = periodMaxCurrentFraction = periodSumOfPositionErrorSquares = periodMaxAbsPositionError = 0.0;
	}
}

// Call this if (and only if) we are in torque mode and want to resume normal movement mode.
// When not called from the closed loop/TMC task, task scheduling should be disabled before calling this.
void ClosedLoop::ExitTorqueMode() noexcept
{
	errorDerivativeFilter.Reset();
	speedFilter.Reset();
	SetTargetToCurrentPosition();
	inTorqueMode = false;
}

#if SUPPORT_DCSERVO
void ClosedLoop::ApplyDcTorque(float torque) noexcept
{
    switch (dcOutputMode)
    {
        case DcServoOutputMode::IoxPwm:
            ApplyDcTorqueIox(torque);
            break;

        case DcServoOutputMode::TmcSinglePhase:
            ApplyDcTorqueTmc(torque);
            break;
    }
}

void ClosedLoop::ApplyDcTorqueIox(float torque) noexcept
{
    const float clamped = constrain<float>(torque, -1.0f, 1.0f);
    SetDcPwm(clamped * 256.0f);
}

void ClosedLoop::ApplyDcTorqueTmc(float torque) noexcept
{
#if SUPPORT_TMC51xx
	// Only write XDIRECT when the TMC has been configured for direct mode.
	// Guards against XDIRECT writes being silently ignored by the chip when GCONF.direct_mode is not set.
	if (SmartDrivers::GetDriverMode(driverNumber) != DriverMode::direct)
	{
		return;
	}

	// Normalize torque [-1, 1] → target current in Amps, capped at dcMaxCurrentTmc
	const float targetCurrent = constrain<float>(torque, -1.0f, 1.0f) * dcMaxCurrentTmc;

	// XDIRECT ±255 = IHOLD current. In direct mode UpdateCurrent() forces IHOLD == IRUN,
	// so GetCurrent() (which returns motorCurrent, used to set both) is the correct scale reference.
	const float fullScaleCurrent = SmartDrivers::GetCurrent(driverNumber) * 0.001f;	// mA → A

	const int16_t regVal = (fullScaleCurrent > 0.0f)
		? (int16_t)constrain<int32_t>(lrintf((targetCurrent / fullScaleCurrent) * 255.0f), -255, 255)
		: (int16_t)0;

	// Update member variables so coil-current telemetry (CL_RECORD_COIL_A/B_CURRENT) reports correctly
	coilA = (dcTmcPhaseSelect == 0) ? regVal : (int16_t)0;
	coilB = (dcTmcPhaseSelect == 0) ? (int16_t)0 : regVal;

	// SetDcPhaseCurrents packs coilA/coilB into the XDIRECT register format and routes
	// through SetXdirect → sets phaseToSet + needToSetCoilCurrents → SPI DMA on next TMC cycle (~80µs)
	SmartDrivers::SetDcPhaseCurrents(driverNumber, coilA, coilB);
#endif
}
#endif
#endif

#if SUPPORT_FOC
void ClosedLoop::ApplyFocTorque(float torqueMagnitude, uint16_t electricalAngle) noexcept
{
	if (focController != nullptr)
	{
		focController->ApplyTorque(torqueMagnitude, electricalAngle);
	}
}

// Project the measured phase currents onto the rotor's d/q axes. In voltage mode this is observation
// only; in current mode the loops feed back from it, which is why it must run before the output is
// decided. See the member declarations in ClosedLoop.h for what the values are evidence of.
void ClosedLoop::MeasureFocDq(uint16_t electricalAngle) noexcept
{
#if SUPPORT_DRV8316_SPI
	float ia, ib, ic;
	FocCurrentSense::GetPhaseCurrents(ia, ib, ic);
	FocController::MeasureDq(ia, ib, ic, electricalAngle, focSenseCorrection, lastFocId, lastFocIq);
#else
	(void)electricalAngle;
	lastFocId = lastFocIq = 0.0f;						// no current sensing hardware on this configuration
#endif
}

// Record the d/q voltages actually applied, for telemetry. Both arrive as bus-voltage fractions.
void ClosedLoop::RecordFocVoltages(float vd, float vq) noexcept
{
	// Report volts where the supply voltage is known, so the numbers can be compared against a scope;
	// fall back to the bus fraction rather than logging a misleading zero when it is not.
	const float toVolts = (focSupplyVoltage > 0.0f) ? focSupplyVoltage : 1.0f;
	lastFocVd = vd * toVolts;
	lastFocVq = vq * toVolts;
}

// Current mode runs only when every precondition for a meaningful d/q frame is met. Each of these has
// already been observed to fail in practice, and a current loop closed through any of them regulates
// confidently onto the wrong axis - worse than not closing one at all.
bool ClosedLoop::FocCurrentModeActive() const noexcept
{
#if SUPPORT_DRV8316_SPI
	return focCurrentKp > 0.0f							// M569.1 F: opt-in, so existing configurations are untouched
		&& focMaxCurrent > 0.0f							// M569.1 H: no demand scale without it
		&& focAlignmentDone
		&& FocCurrentSense::IsSynchronised()			// conversions actually arriving, at the carrier centre
		&& FocCurrentSense::IsCalibrated()				// zero offsets measured, so the readings mean amps
		&& focSenseFrameDirection != 0					// sense frame measured against the drive frame...
		&& focSenseFrameConfidence >= 0.5f;				// ...and the measurement was trustworthy
#else
	return false;
#endif
}

// Whether this tick's d/q measurement may be fed back into the current loops. Separate from
// FocCurrentModeActive(), which asks whether current mode is configured and calibrated at all: this asks
// whether the specific reading in hand is trustworthy right now, and it goes false the moment the
// current sense saturates. Keeping the two apart lets M122 distinguish "never started" from "started and
// dropped out", which are different problems.
bool ClosedLoop::FocMeasurementUsable() const noexcept
{
#if SUPPORT_DRV8316_SPI
	return FocCurrentSense::IsMeasurementFresh();
#else
	return false;
#endif
}

void ClosedLoop::ApplyFocDqVoltage(float vd, float vq, uint16_t electricalAngle) noexcept
{
	if (focController != nullptr)
	{
		focController->ApplyDqVoltage(vd, vq, electricalAngle);
	}
}

float ClosedLoop::GetFocVoltageScale() const noexcept
{
	return (focSupplyVoltage > 0.0f && focVoltageLimit > 0.0f) ? constrain<float>(focVoltageLimit / focSupplyVoltage, 0.0f, 1.0f) : 1.0f;
}

// One electrical revolution in encoder counts. Prefer the value measured by the alignment sweep; fall
// back to the value derived from the configured CPR and pole pair count when we have not measured one
// (not yet aligned, or the measurement was rejected as implausible).
uint32_t ClosedLoop::GetFocCountsPerElecRev() const noexcept
{
	if (focCountsPerElecRev != 0) { return focCountsPerElecRev; }
	if (encoder == nullptr || polePairCount == 0) { return 0; }
	return (uint32_t)((encoder->GetCountsPerStep() * (float)encoder->GetStepsPerRev()) / (float)polePairCount);
}

// Commutation angle for a raw encoder count. Because the alignment sweep settles the rotor's d-axis on
// electrical zero (see alignTargetAngle in SetClosedLoopEnabled), the value returned here IS the rotor's
// electrical position, so the +90 degrees that ApplyTorque()/InversePark() add on top lands the stator
// vector on the q-axis. focEncoderDirection makes this hold on rigs where the encoder counts the
// opposite way to the electrical angle; it used to be assumed +1, which silently inverted the field
// rotation and produced zero-torque traps rather than continuous torque.
uint16_t ClosedLoop::ComputeFocElectricalAngle(int32_t encoderCount) const noexcept
{
	const uint32_t countsPerElecRev = GetFocCountsPerElecRev();
	if (countsPerElecRev == 0) { return 0; }
	int32_t remainder = ((int32_t)focEncoderDirection * encoderCount) % (int32_t)countsPerElecRev;
	if (remainder < 0) { remainder += (int32_t)countsPerElecRev; }
	return (uint16_t)((((uint32_t)remainder * 4096u) / countsPerElecRev) & 4095u);
}

// External gate-driver nFAULT diagnostic input (see FocDriverFaultPin, board config). Not board-standard
// hardware - the SimpleFOCMini's DRV8313 has an open-drain, active-low nFAULT pin with no SPI/register
// interface (unlike the DRV8316 handled via drv8316PollFaults() elsewhere), so this gives the only way to
// see a fault from firmware at all; wire nFAULT to this pin externally when debugging.
/*static*/ void ClosedLoop::InitFocDriverFaultPin() noexcept
{
	SetPinMode(FocDriverFaultPin, INPUT_PULLUP);		// open-drain output on the driver side needs a pullup to read a clean high when not faulted
}

/*static*/ bool ClosedLoop::ReadFocDriverFault() noexcept
{
	return !digitalRead(FocDriverFaultPin);							// active low: pin reads LOW when a fault is asserted
}
#endif

// End
