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
#if SUPPORT_DCSERVO
# include "Encoders/DcServoEncoder.h"
#endif

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
	moveInstance->ResetDriveMovementState(driverNumber, physicalSteps);		// expects physical units directly
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
	float tempTorquePerAmp;
#if SUPPORT_DCSERVO
	uint8_t tempDcOutputMode = (uint8_t)dcOutputMode;
	float tempDcMaxCurrentTmc = dcMaxCurrentTmc;
	uint8_t tempDcTmcPhaseSelect = dcTmcPhaseSelect;
#endif
#if SUPPORT_FOC
	uint8_t tempPolePairCount = polePairCount;
#endif

	// Pull changed parameters
	const bool seenT = parser.GetUintParam('T', tempEncoderType);
	const bool seenC = parser.GetFloatParam('C', tempCPR);
	const bool seenPid = parser.GetFloatParam('R', tempKp) | parser.GetFloatParam('I', tempKi)  | parser.GetFloatParam('D', tempKd) | parser.GetFloatParam('J', tempKpp)
						| parser.GetFloatParam('V', tempKv) | parser.GetFloatParam('A', tempKa);
	const bool seenE = parser.GetFloatArrayParam('E', numThresholds, tempErrorThresholds);
	const bool seenS = parser.GetUintParam('S', tempStepsPerRev);
	const bool seenQ = parser.GetFloatParam('Q', tempTorquePerAmp);
#if SUPPORT_DCSERVO
	const bool seenU = parser.GetUintParam('U', tempDcOutputMode);
	const bool seenW = parser.GetFloatParam('W', tempDcMaxCurrentTmc);
	const bool seenZ = parser.GetUintParam('Z', tempDcTmcPhaseSelect);
#endif
#if SUPPORT_FOC
	const bool seenL = parser.GetUintParam('L', tempPolePairCount);
#endif

	// Report back if no parameters to change
	if (!(seenT || seenC || seenPid || seenE || seenQ || seenS
#if SUPPORT_DCSERVO
		|| seenU || seenW || seenZ
#endif
#if SUPPORT_FOC
		|| seenL
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
			if (encoder->GetType() == EncoderType::dcServo)
			{
				reply.lcatf(", DC output: %s", (dcOutputMode == DcServoOutputMode::IoxPwm) ? "IOX" :
													(dcTmcPhaseSelect == 0) ? "TMC (phase A)" : "TMC (phase B)");
				if (dcOutputMode == DcServoOutputMode::TmcSinglePhase) {
					reply.catf(", max current %.2fA", (double)dcMaxCurrentTmc);
				}
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

	if (seenT && tempEncoderType == EncoderType::dcServo)
	{
		// For DC servo control, we force a 1:1 mapping of steps to quadrature encoder counts.
		// The user's M92 steps/mm should be set to the encoder's quadrature counts/mm.
		tempStepsPerRev = (uint16_t)(tempCPR * 4);
	}
#endif

#if SUPPORT_DCSERVO
	// If changing DC output mode, we need to disable the driver first
	if (seenU && (DcServoOutputMode)tempDcOutputMode != dcOutputMode)
#endif

	if (seenT)
	{
		SetClosedLoopEnabled(ClosedLoopMode::open, reply);		// we need to do this because we are going to mess with the encoder. Do it before we change the closed loop parameters.
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
#endif
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
			// Use our new dedicated DcServoEncoder class
			encoder = new DcServoEncoder((uint32_t)tempCPR, tempStepsPerRev);
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
				motorType = (EncoderType)tempEncoderType;
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
				tuningError = encoder->MinimalTuningNeeded();
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

	// Read the current state of the drive.
	if (encoder->TakeReading())
	{
#if SUPPORT_DCSERVO
		if (encoder->GetType() == EncoderType::dcServo)
		{
			// For DC servo, we handle motion parameter fetching and control inside ControlMotorCurrents
			if (currentMode != ClosedLoopMode::open && tuning == 0 && tuningError == 0 && !stall)
			{
				ControlMotorCurrents(now, timeElapsed);
			}
		}
		else
#endif
#if SUPPORT_FOC
		if (motorType == EncoderType::bldc
			|| motorType == EncoderType::stepperFoc
			|| motorType == EncoderType::hybridStepperFoc)
		{
			// FOC uses pole-pair count (L param) for commutation — basic tuning is not applicable.
			// Mask NeedsBasicTuning out of the gate so a quadrature encoder doesn't block the loop.
			const TuningErrors focTuningError = tuningError & ~TuningError::NeedsBasicTuning;
			if (currentMode != ClosedLoopMode::open && tuning == 0 && focTuningError == 0 && !stall)
			{
				const bool hadMovementBeforeFoc = hasMovementCommand;
				ControlMotorCurrents(now, timeElapsed);
				if (samplingMode == RecordingMode::OnNextMove && hasMovementCommand && !hadMovementBeforeFoc)
				{
					dataCollectionStartTicks = whenNextSampleDue = now;
					samplingMode = RecordingMode::Immediate;
				}
			}
		}
		else
#endif
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
			else if (tuningError == 0)
			{
				currentFraction = ControlMotorCurrents(now, timeElapsed); // otherwise control those motor currents!
			if (inTorqueMode)
			{
				stall = preStall = false;
			}
			else
			{
				// Look for a stall or pre-stall
				const float positionErr = fabsf(currentPositionError);
				if (stall)
				{
					// Reset the stall flag when the position error falls to below half the tolerance, to avoid generating too many stall events
					//TODO do we need a minimum delay before resetting too?
					if (errorThresholds[1] <= 0 || positionErr < errorThresholds[1]/2)
					{
						stall = false;
#if SUPPORT_DCSERVO
						if (encoder->GetType() == EncoderType::dcServo)
						{
							PIDITerm = 0.0f;
							last_vel_error = 0.0f;
							last_filtered_D = 0.0f;
							speedFilter.Reset();
						}
#endif
				}
				}
				else
			{
					stall = errorThresholds[1] > 0 && positionErr > errorThresholds[1];
					if (stall)
					{
						// A stall has just been detected. Stop the motor immediately.
#if SUPPORT_DCSERVO
						if (encoder->GetType() == EncoderType::dcServo)
						{
							// Prevent PID windup and spikes by clearing accumulators
							PIDITerm = 0.0f;
							last_vel_error = 0.0f;
							last_filtered_D = 0.0f;
							// On stall, immediately command zero torque to the motor.
							// This will call the appropriate backend (IOX or TMC) to set output to zero.
							ApplyDcTorque(0.0);
						}
#endif
						// For steppers, the next call to ControlMotorCurrents will be skipped by the !stall check.
						Heat::NewDriverFault();
						}
						else
						{
							preStall = errorThresholds[0] > 0 && positionErr > errorThresholds[0];
						}
					}
				}
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

		if (filterRequested & CL_RECORD_RAW_ENCODER_READING) 	{ sampleBuffer.PutI32(encoder->GetCurrentCount()); }
		if (filterRequested & CL_RECORD_CURRENT_MOTOR_STEPS) 	{ sampleBuffer.PutF32((float)encoder->GetCurrentCount() * encoder->GetStepsPerCount()); }
		if (filterRequested & CL_RECORD_TARGET_MOTOR_STEPS)
		{
#if SUPPORT_DCSERVO
			// For DC servo use the raw physical position from DriveMovement, which is unaffected by the S0/S1
			// direction setting and is directly comparable to CL_RECORD_CURRENT_MOTOR_STEPS.
			if (encoder->GetType() == EncoderType::dcServo)
			{
				sampleBuffer.PutF32(moveInstance->GetTargetMotorStepsPhysical(driverNumber));
			}
			else
#endif
			{
				sampleBuffer.PutF32(mParams.position);
			}
		}
		// For DC servo, logical-space values are sign-flipped when S0 direction is active. Apply dcServoMultiplier to
		// convert them back to physical space so all chart traces are consistent with measured/target position.
#if SUPPORT_DCSERVO
		const float recordMultiplier = (encoder->GetType() == EncoderType::dcServo) ? dcServoMultiplier : 1.0f;
#else
		constexpr float recordMultiplier = 1.0f;
#endif
		if (filterRequested & CL_RECORD_CURRENT_ERROR) 			{ sampleBuffer.PutF32(currentPositionError * recordMultiplier); }
		if (filterRequested & CL_RECORD_PID_CONTROL_SIGNAL)  	{ sampleBuffer.PutF16(PIDControlSignal * recordMultiplier); }
		if (filterRequested & CL_RECORD_PID_P_TERM)
		{
#if SUPPORT_FOC
			if (focController != nullptr) { sampleBuffer.PutF16(focController->lastDutyU); }
			else
#endif
			{ sampleBuffer.PutF16(PIDPTerm * recordMultiplier); }
		}
		if (filterRequested & CL_RECORD_PID_I_TERM)
		{
#if SUPPORT_FOC
			if (focController != nullptr) { sampleBuffer.PutF16(focController->lastDutyV); }
			else
#endif
			{ sampleBuffer.PutF16(PIDITerm * recordMultiplier); }
		}
		if (filterRequested & CL_RECORD_PID_D_TERM)
		{
#if SUPPORT_FOC
			if (focController != nullptr) { sampleBuffer.PutF16(focController->lastDutyW); }
			else
#endif
			{ sampleBuffer.PutF16(PIDDTerm * recordMultiplier); }
		}
		if (filterRequested & CL_RECORD_CURRENT_STEP_PHASE)  	{ sampleBuffer.PutU16(encoder->GetCurrentPhasePosition()); }
		if (filterRequested & CL_RECORD_DESIRED_STEP_PHASE)  	{ sampleBuffer.PutU16(desiredStepPhase); }
		if (filterRequested & CL_RECORD_PHASE_SHIFT)  			{ sampleBuffer.PutF16(vel_measured * recordMultiplier); }
		if (filterRequested & CL_RECORD_COIL_A_CURRENT) 		{ sampleBuffer.PutI16(coilA); }
		if (filterRequested & CL_RECORD_COIL_B_CURRENT) 		{ sampleBuffer.PutI16(coilB); }
		if (filterRequested & CL_RECORD_PID_V_TERM)  			{ sampleBuffer.PutF16(PIDVTerm * recordMultiplier); }
		if (filterRequested & CL_RECORD_PID_A_TERM)  			{ sampleBuffer.PutF16(PIDATerm * recordMultiplier); }
		if (filterRequested & CL_RECORD_PID_J_TERM)				{ sampleBuffer.PutF16(PIDJTerm * recordMultiplier); }
		if (filterRequested & CL_RECORD_MEASURED_VELOCITY)
		{
#if SUPPORT_DCSERVO
			if (encoder->GetType() == EncoderType::dcServo)
			{
				// For DC Servos, convert velocity to mm/sec for charting to make it human-readable.
				// The internal PID loop continues to use counts/tick.
				const float vel_mm_per_sec = (vel_measured * recordMultiplier * StepTimer::StepClockRate) / moveInstance->DriveStepsPerMm(driverNumber);
				sampleBuffer.PutF16(vel_mm_per_sec);
			}
			else
#endif
			{
				sampleBuffer.PutF16(vel_measured);
			}
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
	if (encoder->GetType() == EncoderType::dcServo)
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
		const float multiplier = (moveInstance->GetDirectionValueNoCheck(driverNumber)) ? 1.0f : -1.0f;

		const bool hadMovementCommand = hasMovementCommand;
		hasMovementCommand = moveInstance->GetCurrentMotion(driverNumber, now, mParams);
		if (hasMovementCommand && !hadMovementCommand)
		{
			SetTargetToCurrentPosition();
			PIDITerm = 0.0f;
			last_vel_error = 0.0f;
			last_filtered_D = 0.0f;
			speedFilter.Reset();
			moveInstance->GetCurrentMotion(driverNumber, now, mParams);
		}

		const float targetPhysicalCount = (mParams.position * encoder->GetCountsPerStep()) * multiplier;
		currentPositionError = (targetPhysicalCount - (float)encoder->GetCurrentCount()) * encoder->GetStepsPerCount() * multiplier;
		speedFilter.ProcessReading(encoder->GetCurrentCount() * encoder->GetStepsPerCount(), now);

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

		// Compute electrical angle from encoder position and pole pair count
		const int32_t encoderCount = encoder->GetCurrentCount();
		const uint32_t countsPerElecRev = (uint32_t)((encoder->GetCountsPerStep() * (float)encoder->GetStepsPerRev()) / (float)polePairCount);
		uint16_t electricalAngle = 0;
		if (countsPerElecRev > 0)
		{
			int32_t remainder = encoderCount % (int32_t)countsPerElecRev;
			if (remainder < 0) { remainder += (int32_t)countsPerElecRev; }
			electricalAngle = (uint16_t)(((uint32_t)remainder * 4096u) / countsPerElecRev);
		}

		// Torque magnitude in [-1, 1]: scale control signal by multiplier and normalise
		const float torqueMagnitude = constrain<float>((PIDControlSignal * multiplier) / 256.0f, -1.0f, 1.0f);
		ApplyFocTorque(torqueMagnitude, electricalAngle);

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
	reply.catf(", encoder type %s", GetEncoderType().ToString());
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
	}

	//DEBUG
	//reply.catf(", event status 0x%08" PRIx32 ", TCC2 CTRLA 0x%08" PRIx32 ", TCC2 EVCTRL 0x%08" PRIx32, EVSYS->CHSTATUS.reg, QuadratureTcc->CTRLA.reg, QuadratureTcc->EVCTRL.reg);
}

/*static*/ void ClosedLoop::Diagnostics(const StringRef& reply) noexcept
{
	for (size_t i = 0; i < NumDrivers; ++i)
	{
		moveInstance->ClosedLoopDiagnostics(i, reply);
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
		SetTargetToCurrentPosition();

		// Set the target position to the current position
		ResetError();													// this calls ReadState again and sets up targetMotorSteps

		moveInstance->ResetPhaseStepMonitoringVariables();				// to avoid getting stupid values
		moveInstance->ResetPhaseStepControlLoopCallTime();				// to avoid huge integral term windup
	}

	// If we are disabling closed loop mode, we should ideally send steps to get the microstep counter to match the current phase here
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
	SmartDrivers::SetDcPhaseCurrents(driverNumber, 100, 0);
    return;  // Skip the rest of the function
	// Convert the normalized torque [-1.0, 1.0] to a target current
	const float targetCurrent = constrain<float>(torque, -1.0f, 1.0f) * dcMaxCurrentTmc;

	// Get the configured run current for the driver, which is used as the reference for XDIRECT
	const float fullScaleCurrent = SmartDrivers::GetCurrent(driverNumber) * 0.001f;  // Convert mA to A

	// Calculate the 9-bit signed value for the XDIRECT register.
	// A value of 255 corresponds to the current set by IHOLD. We assume IHOLD is set to the same as IRUN.
	const int16_t currentRegisterValue = (fullScaleCurrent > 0.0f)
											? lrintf((targetCurrent / fullScaleCurrent) * 255.0f)
											: 0;

	if (dcTmcPhaseSelect == 0)
	{
		coilA = currentRegisterValue;
		coilB = 0;
	}
	else
	{
		coilA = 0;
		coilB = currentRegisterValue;
	}
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
#endif

// End
