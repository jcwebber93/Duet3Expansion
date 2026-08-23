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
#  include "FocCurrentSense.h"
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
	// Gate-driver / current-sense hardware state, reported as a separate M122 part because each
	// part shares one 500-char reply buffer and silently truncates when it overflows.
	void InstanceDriverDiagnostics(size_t driver, const StringRef& reply) noexcept;

	// Methods called by the motion system
	EncoderType GetEncoderType() const noexcept
	{
		return (encoder == nullptr) ? EncoderType::none : encoder->GetType();
	}

	bool IsDcServoMode() const noexcept { return isDcServoMode; }

#if SUPPORT_FOC
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
	static void DriverDiagnostics(const StringRef& reply) noexcept;

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

	float focSupplyVoltage = 0.0f;								// nominal motor supply voltage in volts (M569.1 N parameter); 0 = not configured
	float focVoltageLimit = 0.0f;								// FOC voltage limit in volts (M569.1 O parameter); 0 = not configured


	// Encoder-to-electrical-angle relationship, MEASURED by the alignment
	// 0 in focCountsPerElecRev means "not measured" - fall back to the value derived from C and L.
	int8_t focEncoderDirection = 1;								// +1 or -1; sign of d(electrical angle)/d(encoder count)
	uint32_t focCountsPerElecRev = 0;							// measured encoder counts per electrical revolution; 0 = not measured

	uint16_t lastFocElectricalAngle = 0;
	float lastFocTorqueMagnitude = 0.0f;
	bool lastFocDriverFault = false;

	// Rotor-frame view of the measured phase currents, refreshed every control tick from
	// FocController::MeasureDq(), plus the d/q voltages that produced them.
	// Voltages are in volts when M569.1 N (supply voltage) is configured, otherwise a fraction of the bus.
	float lastFocId = 0.0f;
	float lastFocIq = 0.0f;
	float lastFocVd = 0.0f;
	float lastFocVq = 0.0f;

	// Orientation of the current-sense frame relative to the drive frame, measured during the alignment
	// sweep - the one place where the commanded electrical angle is known independently of the encoder
	// and the torque is constant. See the sweep in SetClosedLoopEnabled() for why nowhere else will do.
	//
	// direction: +1 the two frames agree, -1 the sense frame is mirrored (phase leads or ISEN inputs in
	// opposite rotational order), 0 not measured. offsetDeg is the residual angle between the measured
	// current vector and the commanded voltage vector, which should be small. confidence is the vector
	// mean length over the sweep, 0..1; near 1 means every sample agreed, near 0 means the measurement is
	// noise and neither hypothesis should be believed.
	int8_t focSenseFrameDirection = 0;
	uint16_t focSenseFrameSamples = 0;
	uint16_t focSenseFrameClipped = 0;							// sweep samples rejected for failing Kirchhoff, i.e. saturated
	float focSenseFrameOffsetDeg = 0.0f;
	float focSenseFrameConfidence = 0.0f;

	// Torque the alignment sweep actually used, after the current-limiting ramp backed it off from the
	// W/voltage-scale ceiling. 0 means the ramp did not run (no current sensing).
	float focAlignTorqueUsed = 0.0f;
	uint8_t focAlignCorrections = 0;
	uint32_t focMeasuredElecRevCounts = 0;					// what the sweep measured, 0 if it was rejected							// settle-and-correct passes the alignment needed

	// The correction derived from the above and applied to every d/q measurement, plus what was left
	// over after snapping to the 60-degree grid. The residual is the winding's load angle and is
	// deliberately NOT corrected; a large one means the snap picked the wrong grid point.
	FocController::SenseFrameCorrection focSenseCorrection;
	float focSenseFrameResidualDeg = 0.0f;

	// Current-mode (d/q) control. Off unless M569.1 F gives a non-zero gain AND the current sense is
	// calibrated AND the sense frame has been measured - see FocCurrentModeActive(). Everything falls back
	// to voltage mode otherwise, which is what boards without current sensing always do.
	//
	// Gains are shared between the d and q axes - same winding, same L and R. Units are bus-voltage
	// fraction per amp (Kp) and per amp-second (Ki); for bandwidth w rad/s on a bus of V volts,
	// Kp = L*w/V and Ki = R*w/V, with Ki/Kp = R/L placing the PI zero on the plant pole.
	//
	// DO NOT DETUNE THESE FOR SAFETY. Near-unity proportional loop gain is correct for a current loop;
	// slowing the inner loop breaks the cascade's timescale separation and causes runaway, not calm.
	// Background: docs/foc-current-control.md#tuning
	//
	float focCurrentKp = 0.0f;
	float focCurrentKi = 0.0f;
	float focMaxCurrent = 0.0f;									// amps; the current-mode analogue of W

	float focIdIntegral = 0.0f;									// integrator state, in bus-voltage fraction
	float focIqIntegral = 0.0f;
	float lastFocIqTarget = 0.0f;								// for diagnostics and telemetry
	bool focCurrentModeRunning = false;							// whether the last tick actually ran the loops
	uint32_t focCurrentModeDropouts = 0;						// times the loops fell back on an unusable measurement
	uint32_t focCurrentTrips = 0;								// times the measured current exceeded the trip level

	// Output ceiling used when current mode is configured but its measurement is unusable. Re-seeded from
	// the regulated output on every good tick and decayed while the measurement is missing, so the drive
	// resumes from a level known to be safe and goes quiet if the fault persists - instead of being handed
	// full voltage authority at the exact moment nothing is limiting the current.
	// Background: docs/foc-current-control.md#fallback
	float focFallbackCeiling = 0.0f;

	static constexpr float CurrentTripFactor        = 1.5f;		// x focMaxCurrent before the ceiling is squeezed
	static constexpr float OverTripOutputFraction   = 0.25f;	// ceiling applied while over the trip
	static constexpr float FallbackHeadroom         = 1.10f;	// margin over the last regulated output
	static constexpr float FallbackDecayStale       = 0.100f;	// s; a brief gap barely dents the output
	static constexpr float FallbackDecaySaturated   = 0.010f;	// s; already over range, so collapse fast
	static constexpr float FallbackFloorFraction    = 0.02f;	// below this fraction of maxOutput, stop driving

	// Alignment current limiting. The alignment sweep has a current FLOOR as well as a ceiling - it has to
	// drag the rotor through a full electrical revolution against cogging and inertia, and a sweep that
	// fails to move the rotor sets TuningError::TooLittleMotion, which gates the control law entirely.
	// Every constant below exists to stop the correction undershooting.
	// Background: docs/foc-current-sense.md#alignment-overcurrent
	static constexpr float AlignMaxCurrent          = 3.0f;		// A, upper bound whatever the motor is configured for
	static constexpr float AlignCurrentHeadroom     = 1.5f;		// x focMaxCurrent: alignment is a brief static hold
	static constexpr float AlignMinTorque           = 0.02f;	// never reduce below this fraction of the bus
	static constexpr float AlignCorrectionDeadband  = 1.25f;	// only correct when clearly over target, not to chase it
	static constexpr float AlignMinScalePerPass     = 0.60f;	// bound one pass, so a bad reading cannot collapse it
	static constexpr float AlignMinRetained         = 0.40f;	// floor across all passes, relative to the ramp result
	static constexpr float RampBlindFraction        = 0.25f;	// used when the coarse ramp never reads over target
	static constexpr unsigned int MaxAlignCorrections = 2;		// settle-and-correct passes allowed

	// Within this much of the C/L value the two are taken to agree and the exact derived period is used.
	// Outside it the configuration is suspect and the sweep, biased as it is, becomes the better guess.
	static constexpr uint32_t ElecPeriodAgreementPercent = 10;

	// True when the d/q loops may be run: gains set, hardware present, offsets calibrated and the sense
	// frame measured. Closing a loop on an uncalibrated frame would regulate onto the wrong axis.
	bool FocCurrentModeActive() const noexcept;

	// Whether the current d/q reading is fresh enough to feed back from. False on a saturated or
	// incoherent scan, which is what stops a saturation event latching the loops at full output.
	bool FocMeasurementUsable() const noexcept;

	// Whether the sense amplifiers are clipping. Unusable as a reading, but it is a positive report of
	// an overcurrent, so it drives the output down harder than a merely stale measurement does.
	bool FocMeasurementSaturated() const noexcept;
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
	float	PIDVTerm;									// Velocity feedforward term
	float	PIDATerm;									// Acceleration feedforward term
	float	PIDControlSignal;							// The overall signal from the PID controller
	float	PIDJTerm;									// Position Proportional term
	float 	vel_measured;
	float	last_vel_error = 0.0;
	float last_filtered_D = 0.0;


	uint16_t desiredStepPhase = 0;						// The desired position of the motor
	uint16_t phaseOffset = 0;							// The amount by which the phase should be offset when in semi-open-loop mode
	int16_t coilA = 0;									// The current to run through coil A
	int16_t coilB = 0;									// The current to run through coil B
														// (both stay 0 for FOC/BLDC drives, which drive the phases via FocController instead)

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

	// Update the stall/pre-stall flags from currentPositionError and take the immediate stop action if a
	// stall has just been detected. Called once per control tick, for every drive type
	void UpdateStallDetection() noexcept;
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

	// Refresh lastFocId/lastFocIq from the latest phase-current measurement. Must be called before the
	// output is computed on any tick where the current loops are running.
	void MeasureFocDq(uint16_t electricalAngle) noexcept;

	// Record the applied d/q voltages (bus-voltage fractions) into lastFocVd/lastFocVq for telemetry.
	void RecordFocVoltages(float vd, float vq) noexcept;

	// Apply a d/q voltage vector through the output stage. Both are bus-voltage fractions.
	void ApplyFocDqVoltage(float vd, float vq, uint16_t electricalAngle) noexcept;

	// One electrical revolution in encoder counts: the value measured by the alignment sweep if we have
	// one, otherwise derived from the configured CPR and pole pair count. Returns 0 if neither is usable.
	// The electrical period as an exact rational in encoder counts, so the commutation angle never
	// carries a rounding error that integrates with distance.
	// Background: docs/foc-commutation.md#counts-per-elec-rev
	void GetFocElecPeriod(uint32_t& numerator, uint32_t& denominator) const noexcept;

	uint32_t GetFocCountsPerElecRev() const noexcept;

	// Commutation angle (0..4095) for a raw encoder count, applying the measured encoder direction.
	// Single definition so the runtime and alignment paths cannot disagree.
	uint16_t ComputeFocElectricalAngle(int32_t encoderCount) const noexcept;

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
