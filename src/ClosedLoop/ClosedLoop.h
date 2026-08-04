/*
 * ClosedLoop.h
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_CLOSEDLOOP_H_
#define SRC_CLOSEDLOOP_CLOSEDLOOP_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

# include <CanMessageFormats.h>
# include <General/NamedEnum.h>
# include <Movement/StepTimer.h>
# include <ClosedLoop/Trigonometry.h>
# include <SPI/SharedSpiDevice.h>
# include "DerivativeAveragingFilter.h"
# include "TuningErrors.h"
# include "SampleBuffer.h"
# include "Encoders/Encoder.h"
# if SUPPORT_FOC
#  include "FocController.h"
# endif
# if SUPPORT_DRV8316_SPI
#  include "DRV8316.h"
# endif

constexpr float MaxSafeBacklash = 0.22;					// the maximum backlash in full steps that we can use - error if there is more
constexpr float MaxGoodBacklash = 0.15;					// the maximum backlash in full steps that we are happy with - warn if there is more
constexpr unsigned int LinearEncoderIncreaseFactor = 4;	// this should be a power of 2. Allowed backlash is increased by this amount for linear composite encoders.
constexpr float VelocityLimitGainFactor = 5.0;			// the gain of the P loop when in torque mode

class Encoder;
class SpiEncoder;
class CanMessageGenericParser;

// Struct to pass data back to the ClosedLoop module
struct MotionParameters
{
	float position = 0.0;
	float speed = 0.0;
	float acceleration = 0.0;
};

enum class ClosedLoopMode
{
	open = 0,
	closed,
	assistedOpen
};

enum class DcServoOutputMode : uint8_t
{
	IoxPwm = 0,
	TmcSinglePhase = 1
};

class ClosedLoop
{
public:
	// Constants and variables that are used by both the ClosedLoop and the Tuning modules

	// Tuning manoeuvres
	static constexpr uint8_t BASIC_TUNING_MANOEUVRE 				= 1u << 0;		// this measures the polarity, check that the CPR looks OK, and for relative encoders sets the zero position
	static constexpr uint8_t ENCODER_CALIBRATION_MANOEUVRE 			= 1u << 1;		// this calibrates an absolute encoder
	static constexpr uint8_t ENCODER_CALIBRATION_CHECK				= 1u << 2;		// this checks the calibration
	static constexpr uint8_t STEP_MANOEUVRE 						= 1u << 6;		// this does a sudden step change in the requested position for PID tuning

#if 0	// The remainder are not currently implemented
	constexpr uint8_t CONTINUOUS_PHASE_INCREASE_MANOEUVRE 	= 1u << 5;
	constexpr uint8_t ZIEGLER_NICHOLS_MANOEUVRE 			= 1u << 7;
#endif

	// Closed loop public methods
	void InitInstance() noexcept;

	GCodeResult ProcessM569Point1(CanMessageGenericParser& parser, const StringRef& reply) noexcept;
	GCodeResult ProcessM569Point4(CanMessageGenericParser& parser, const StringRef& reply) noexcept;
	GCodeResult ProcessM569Point5(const CanMessageStartClosedLoopDataCollection&, const StringRef& reply) noexcept;
	GCodeResult ProcessM569Point6(CanMessageGenericParser& parser, const StringRef& reply) noexcept;
	void UpdateStandstillCurrent() noexcept;

	const char *_ecv_array GetModeText() const noexcept;
	void InstanceDiagnostics(size_t driver, const StringRef& reply) noexcept;

	// Methods called by the motion system
	EncoderType GetEncoderType() const noexcept
	{
		return (encoder == nullptr) ? EncoderType::none : encoder->GetType();
	}

	bool IsDcServoMode() const noexcept { return isDcServoMode; }

#if SUPPORT_FOC
	// True for a BLDC/FOC-controlled drive. Like DC servo, this drive's currentMotorPosition/target
	// bookkeeping is physical and direction-independent (electrical commutation angle is derived
	// straight from live encoder position, not from accumulated direction-signed step count), so it
	// must NOT be inverted on an S0/S1 direction change the way a classic stepper's step count is -
	// see Move::SetDirectionValue().
	bool IsFocMode() const noexcept { return motorType == EncoderType::bldc || motorType == EncoderType::stepperFoc || motorType == EncoderType::hybridStepperFoc; }
#endif

	void InstanceControlLoop(StepTimer::Ticks now, StepTimer::Ticks timeElapsed) noexcept;
	StandardDriverStatus ReadLiveStatus() const noexcept;
	bool IsClosedLoopEnabled() const noexcept;
	bool SetClosedLoopEnabled(ClosedLoopMode mode, const StringRef &reply) noexcept;
	void DriverSwitchedToClosedLoop() noexcept;
	void ResetError() noexcept;
	bool OkayToSetDriverIdle() const noexcept;
	StandardDriverStatus ModifyDriverStatus(StandardDriverStatus originalStatus) const noexcept;
	void GetStatistics(ClosedLoopStatus& stat) noexcept;

	// Methods called by the encoders
	static void EnableEncodersSpi() noexcept;
	static void DisableEncodersSpi() noexcept;

	static void Init() noexcept;
	static void Diagnostics(const StringRef& reply) noexcept;

	// Functions run by tasks
	[[noreturn]] void DataTransmissionTaskLoop() noexcept;
	[[noreturn]] void EncoderCalibrationTaskLoop() noexcept;

private:
	// Constants private to this module
	static constexpr unsigned int DerivativeFilterSize = 8;			// The range of the error derivative filter (use a power of 2 for efficiency)
	static constexpr unsigned int SpeedFilterSize = 8;				// The range of the speed filter (use a power of 2 for efficiency)
	static constexpr unsigned int tuningStepsPerSecond = 2000;		// the rate at which we send 1/256 microsteps during tuning, slow enough for high-inertia motors
	static constexpr StepTimer::Ticks stepTicksPerTuningStep = StepTimer::StepClockRate/tuningStepsPerSecond;
	static constexpr StepTimer::Ticks stepTicksBeforeTuning = StepTimer::StepClockRate/10;
																	// 1/10 sec delay between enabling the driver and starting tuning, to allow for brake release and current buildup
	static constexpr StepTimer::Ticks DataCollectionIdleStepTicks = StepTimer::StepClockRate/200;
																	// start collecting tuning data 5ms before the start of the tuning move
	static constexpr float DefaultHoldCurrentFraction = 0.25;		// the minimum fraction of the requested current that we apply when holding position
	static constexpr float DefaultTorquePerAmp = 1.0;				// the torque per amp of motor current

	static constexpr float PIDIlimit = 80.0;

	static constexpr size_t driverNumber = 0;						// the driver number of this instance, can be changed to a variable if we ever support more than one closed loop driver on a board

	// Methods used only by closed loop and by the tuning module
	void SetMotorPhase(uint16_t phase, float magnitude) noexcept;
	void FinishedBasicTuning() noexcept;
																// call this when we have stopped basic tuning movement and are ready to switch to closed loop control
	void ReadyToCalibrate(bool store) noexcept;					// call this when encoder calibration has finished collecting data
	void AdjustTargetMotorSteps(float amount) noexcept;			// called by tuning to execute a step
	void ExitTorqueMode() noexcept;

	// Methods in the tuning module
	void PerformTune() noexcept;

	// Enumeration of closed loop recording modes
	enum RecordingMode : uint8_t
	{
		None = 0,			// not collecting data
		Immediate,			// collecting data now
		OnNextMove,			// collect data when the next movement command starts executing
		SendingData			// finished collecting data but still sending it to the main board
	};

	Encoder *encoder = nullptr;									// Pointer to the encoder object in use
	volatile uint8_t tuning = 0;								// Bitmask of any tuning manoeuvres that have been requested
	TuningErrors tuningError;									// Flags for any tuning errors

	// Control variables, set by the user to determine how the closed loop controller works
	ClosedLoopMode currentMode = ClosedLoopMode::open;			// which mode the driver is in

	// Holding current, and variables derived from it
	float 	holdCurrentFraction = DefaultHoldCurrentFraction;	// The minimum holding current when stationary
	float	torquePerAmp = DefaultTorquePerAmp;					// the torque per amp of configured current
	float 	Kp = 30.0;											// The proportional constant for the PID controller
	float 	Ki = 0.0;											// The proportional constant for the PID controller
	float 	Kd = 0.0;											// The proportional constant for the PID controller
	float	Kv = 1000.0;										// The velocity feedforward constant
	float	Ka = 0.0;											// The acceleration feedforward constant
	float	Kpp = 1.0;											// The P for position

	// DC Servo specific
	static constexpr float MaxDcServoTmcCurrent = 4.5;			// The absolute maximum current in Amps for the TMC DC servo output mode
	DcServoOutputMode dcOutputMode = DcServoOutputMode::IoxPwm;
	float dcMaxCurrentTmc = 1.0;								// Max current in Amps for TMC DC servo output mode
	uint8_t dcTmcPhaseSelect = 0;								// 0 for phase A, 1 for phase B
	float dcServoMultiplier = 1.0f;								// +1 or -1 per S0/S1 direction setting; persisted so CollectSample can convert logical→physical space
	bool isDcServoMode = false;									// true when controlling a DC servo regardless of encoder type

#if SUPPORT_FOC
	// BLDC/FOC specific
	FocController *focController = nullptr;						// Owns the 3-phase PWM output; created when encoder type is set to bldc
	uint8_t polePairCount = 1;									// Number of electrical pole pairs (M569.1 L parameter)
	EncoderType motorType = EncoderType::none;					// The configured motor type (T param); separate from encoder->GetType() which reflects the sensor
	bool focAlignmentDone = false;								// true once the startup alignment dwell has completed
	float focMaxTorque = 1.0f;									// Peak torque fraction [0.0, 1.0] applied to FOC output (M569.1 W parameter)
	float focMultiplier = 1.0f;									// +1 or -1 per S0/S1 direction setting; persisted so CollectSample can convert logical→physical space

	// Voltage scaling: torqueMagnitude (a [-1,1] fraction) is otherwise applied directly as a duty-cycle
	// fraction of the FULL supply voltage (see FocController::ApplyTorque), unlike SimpleFOC's explicit
	// voltage_limit/voltage_power_supply split. When both are configured (M569.1 N = supply volts,
	// O = voltage limit in volts), torqueMagnitude is scaled by focVoltageLimit/focSupplyVoltage before
	// being sent to the driver, so W/running torque and the alignment pull can be expressed in real,
	// predictable volts instead of an opaque 0..1 fraction of an unstated supply voltage. Zero means
	// "not configured" - falls back to the pre-existing unscaled behaviour.
	float focSupplyVoltage = 0.0f;								// nominal motor supply voltage in volts (M569.1 N parameter); 0 = not configured
	float focVoltageLimit = 0.0f;								// FOC voltage limit in volts (M569.1 O parameter); 0 = not configured

	// Velocity ceiling for the outer position loop's vel_target (PIDJTerm+PIDVTerm+PIDATerm), mirroring
	// SimpleFOC's P_angle.limit/velocity_limit (see FOCMotor.cpp: shaft_velocity_sp is constrained to
	// +-velocity_limit immediately, every call). Without this, vel_target - dominated by the unbounded
	// Kpp*currentPositionError term - only grows as large as the accumulated position error, so a stalled
	// rotor only receives a strong torque demand well AFTER a large error has built up, rather than an
	// immediate, decisive one from the first tick. Confirmed directly: a SimpleFOC capture on the same
	// motor/driver showed vel_target saturating to its limit instantly on a large step, while the
	// Duet3Expansion equivalent (PIDJTerm) climbed smoothly over ~140ms before reaching a comparable
	// magnitude - matching the "torque ramps up, then faults" pattern seen in every reproduced stall.
	// Repurposes M569.1 Q (torquePerAmp), which has no effect on FOC/BLDC drives (only meaningful for
	// SUPPORT_TMC51xx torque-mode current scaling - see ControlMotorCurrents()'s torque-mode branch).
	// Units: steps/second (converted internally to the steps-per-step-clock-tick units vel_target/
	// vel_measured actually use). 0 = not configured, falls back to the pre-existing unclamped behaviour.
	float focVelocityLimit = 0.0f;								// M569.1 Q parameter (FOC drives only), steps/sec; 0 = not configured

	// Continuous slew-rate limit on commanded torque, applied every control tick (not just after
	// alignment). Bounds how fast torqueMagnitude can change in torque-fraction-per-second, which in
	// turn bounds phase current di/dt regardless of how large the raw PID output jumps (e.g. from a
	// sudden hand-induced position error). This mirrors SimpleFOC's PID output_ramp (volts/sec on Uq),
	// which is the mechanism that let the same DRV8313 driver run fault-free under SimpleFOC even with
	// comparably aggressive manual disturbances - the port previously had no equivalent, only a fixed
	// magnitude clamp (focMaxTorque) that let torqueMagnitude jump to full scale within a single ~80us
	// control tick.
	// Disabled by default: measured against real closed-loop logs, this rate (4.0 = 0..1.0 in 250ms) made
	// torqueMagnitude climb far too slowly to reach breakaway torque from a dead stop before the target
	// had already moved further away, causing motion to reliably stall after a few hundred encoder counts
	// - a symptom absent in both open-loop (assistedOpen, which applies a constant unramped magnitude) and
	// in the SimpleFOC reference config (whose own output_ramp equivalent, when used at all, is ~20x less
	// restrictive and off by default). Left in place and re-enable-able via focTorqueRampEnabled in case a
	// slew limit is wanted again for fault mitigation, but it must not be on by default.
	static constexpr float focTorqueRampRate = 4.0f;				// max change in torque fraction per second (~4.0 = 0..1.0 in 250ms), only used if focTorqueRampEnabled
	bool focTorqueRampEnabled = false;							// gate for the torque slew-rate limiter; off by default, see comment above
	float focAppliedTorque = 0.0f;								// last torque fraction actually sent to ApplyFocTorque(), for slew-limiting the next call

	// Electrical-zero verification/correction. The alignment dwell establishes electricalAngle=0 by
	// re-zeroing the encoder at wherever the rotor settles under a fixed-angle-0 pull - this makes the
	// angle formula self-consistent (encoderCount=0 maps to electricalAngle=0) but never confirms that
	// commanding a PURE q-axis torque (90 electrical degrees from that zero) actually produces a
	// torque-maximising, rotor-turning vector rather than one that's rotated towards the (non-torque-
	// producing) d-axis by some fixed error - which SimpleFOC's independent zero_electric_angle
	// calibration (drive to a KNOWN angle, read back the sensor) avoids by construction. If the
	// verification pulse below finds a mismatch, this stores an additive correction so electricalAngle
	// values used for commutation are shifted to genuinely align q-axis commands with the rotor's D-axis.
	uint16_t focElectricalAngleOffset = 0;						// additive correction (0..4095) applied to all commutation angles after alignment

	// Debug snapshot of the last electricalAngle/torqueMagnitude actually passed to ApplyFocTorque(),
	// for M122 diagnostics - lets us see the raw inputs to the SVPWM output stage directly instead of
	// inferring them from PIDControlSignal/duty cycle.
	uint16_t lastFocElectricalAngle = 0;
	float lastFocTorqueMagnitude = 0.0f;
	float lastFocVelTargetRaw = 0.0f;							// TEMPORARY DEBUG: raw vel_target before the focVelocityLimit clamp, for diagnosing the Q4000 zero-torque issue - remove once resolved

	// Live external gate-driver nFAULT pin state (see FocDriverFaultPin, board config), sampled every
	// control tick and logged via CollectSample() so a fault event (e.g. the SimpleFOCMini's DRV8313
	// tripping OCP) can be correlated directly against the exact sample/torque/angle it occurred at,
	// rather than inferred after the fact from a full power-cycle being needed to recover. true = fault
	// pin currently reads active (driver is faulted); the pin is open-drain active-low, so this already
	// accounts for the inversion (see InitFocDriverFaultPin()/ReadFocDriverFault() in ClosedLoop.cpp).
	bool lastFocDriverFault = false;
#endif

#if SUPPORT_DRV8316_SPI
	DRV8316 *drv8316 = nullptr;
	uint8_t drv8316PollCounter = 0;								// incremented each control tick; faults polled every 125 ticks (~10 ms)
#endif

	float 	errorThresholds[2];									// The error thresholds. [0] is pre-stall, [1] is stall

	float torqueModeCommandedCurrentFraction = 0.0;		// when in torque mode, the requested torque
	float torqueModeMaxSpeed = 0.0;						// when in torque mode, the maximum speed. Zero or negative means no limit.

	// Working variables
	// These variables are all used to calculate the required motor currents. They are declared here so they can be reported on by the data collection task
	MotionParameters mParams;							// the target position, speed and acceleration
	float currentPositionError;							// the current position error in full steps
	float periodMaxAbsPositionError = 0.0;				// the maximum value of the absolute position error
	float periodSumOfPositionErrorSquares = 0.0;		// used to calculate the RMS error
	float periodMaxCurrentFraction = 0.0;				// the maximum current fraction over this period
	float periodSumOfCurrentFractions = 0.0;			// used to calculate the average current fraction
	unsigned int periodNumSamples = 0;					// how many samples are in sumOfPositionErrorSquares

	float 	PIDPTerm;									// Proportional term
	float 	PIDITerm = 0.0;								// Integral accumulator
	float 	PIDDTerm;									// Derivative term
	float	PIDVelITerm = 0.0;							// Velocity integral accumulator
	float	PIDVTerm;									// Velocity feedforward term
	float	PIDATerm;									// Acceleration feedforward term
	float	PIDControlSignal;							// The overall signal from the PID controller
	float	PIDJTerm;									// P Pos term
	float 	vel_measured;
	float	last_vel_error = 0.0;
	float last_filtered_D = 0.0;


	uint16_t desiredStepPhase = 0;						// The desired position of the motor
	uint16_t phaseOffset = 0;							// The amount by which the phase should be offset when in semi-open-loop mode
	int16_t coilA;										// The current to run through coil A
	int16_t coilB;										// The current to run through coil A

	bool	hasMovementCommand = false;					// true if a regular movement command is being executed
	bool	inTorqueMode = false;
	bool	torqueModeDirection;
	bool 	stall = false;								// Has the closed loop error threshold been exceeded?
	bool 	preStall = false;							// Has the closed loop warning threshold been exceeded?

	// Basic tuning synchronisation
	volatile bool basicTuningDataReady = false;

	// Encoder calibration synchronisation
	enum class CalibrationState : uint8_t { notReady = 0, dataReady, complete };
	volatile CalibrationState calibrationState = CalibrationState::notReady;
	volatile bool calibrateNotCheck = false;
	volatile TuningErrors calibrationErrors;

	StepTimer::Ticks whenLastTuningStepTaken;			// when the control loop last called the tuning code

	// Data collection variables
	// Input variables
	volatile RecordingMode samplingMode = RecordingMode::None;	// What mode did they request? Volatile because we care about when it is written.
	uint8_t  movementRequested;									// Which calibration movement did they request? 0=none, 1=polarity, 2=continuous
	uint32_t filterRequested;									// What filter did they request?
	volatile uint16_t samplesRequested;							// The number of samples requested

	// Derived variables
	std::atomic<uint16_t> samplesCollected = 0;
	std::atomic<uint16_t> samplesSent = 0;
	bool sampleBufferOverflowed = false;						// set if the buffer is full when we need to store a sample
	StepTimer::Ticks dataCollectionStartTicks;					// At what tick did data collection start?
	StepTimer::Ticks dataCollectionIntervalTicks;				// the requested interval between samples
	StepTimer::Ticks whenNextSampleDue;							// when it will be time to take the next sample

	DerivativeAveragingFilter<DerivativeFilterSize> errorDerivativeFilter;	// An averaging filter to smooth the derivative of the error
	DerivativeAveragingFilter<SpeedFilterSize> speedFilter;		// An averaging filter to smooth the actual speed

	static SampleBuffer sampleBuffer;							// buffer for collecting samples - shared between all drives if we have more than one

	// Return true if we are currently collecting data or primed to collect data or finishing sending data
	inline bool CollectingData() noexcept { return samplingMode != RecordingMode::None; }

	void CollectSample() noexcept;
	float ControlMotorCurrents(StepTimer::Ticks now, StepTimer::Ticks ticksSinceLastCall) noexcept;
	void StartTuning(uint8_t tuningType) noexcept;
	GCodeResult ProcessBasicTuningResult(const StringRef& reply) noexcept;
	GCodeResult ProcessCalibrationResult(const StringRef& reply) noexcept;
	void ReportTuningErrors(TuningErrors tuningErrorBitmask, const StringRef& reply) noexcept;
	void SetTargetToCurrentPosition() noexcept;
	void CreateCalibrationTask() noexcept;

	// Tuning methods
#if SUPPORT_DCSERVO
	void InitDcPwm() noexcept;
	void SetDcPwm(float controlSignal) noexcept;
	void ApplyDcTorque(float torque) noexcept;
	void ApplyDcTorqueIox(float torque) noexcept;
	void ApplyDcTorqueTmc(float torque) noexcept;
#endif

#if SUPPORT_FOC
	void ApplyFocTorque(float torqueMagnitude, uint16_t electricalAngle) noexcept;

	// Returns the multiplier to apply to a torque-fraction command to respect focVoltageLimit/focSupplyVoltage,
	// or 1.0 (no scaling) if either is not configured (0.0). Defined in ClosedLoop.cpp.
	float GetFocVoltageScale() const noexcept;

	static void InitFocDriverFaultPin() noexcept;					// one-time GPIO setup for FocDriverFaultPin
	static bool ReadFocDriverFault() noexcept;						// true if the external gate driver's nFAULT is currently asserted
#endif

	bool BasicTuning(bool firstIteration) noexcept;
	bool EncoderCalibration(bool firstIteration) noexcept;
	bool Step(bool firstIteration) noexcept;
};

// Return true if the driver is in closed loop or assisted open loop mode
inline bool ClosedLoop::IsClosedLoopEnabled() const noexcept
{
	return currentMode != ClosedLoopMode::open;
}

// The encoder uses the standard shared SPI device, so we don't need to enable/disable it
inline void ClosedLoop::EnableEncodersSpi() noexcept { }
inline void ClosedLoop::DisableEncodersSpi() noexcept { }

# endif

#endif /* SRC_CLOSEDLOOP_CLOSEDLOOP_H_ */
