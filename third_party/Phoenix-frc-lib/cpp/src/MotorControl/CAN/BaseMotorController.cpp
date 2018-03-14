#include "ctre/phoenix/MotorControl/CAN/BaseMotorController.h"
#include "ctre/phoenix/MotorControl/SensorCollection.h"
#include "ctre/phoenix/CCI/MotController_CCI.h"
#include "ctre/phoenix/LowLevel/MotControllerWithBuffer_LowLevel.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol::lowlevel;

//--------------------- Constructors -----------------------------//
/**
 *
 * Constructor for motor controllers.
 * @param arbId
 */
BaseMotorController::BaseMotorController(int arbId) {
	m_handle = c_MotController_Create1(arbId);
	_arbId = arbId;

	_sensorColl = new motorcontrol::SensorCollection((void*) m_handle);
}
/**
 *
 * Destructor
 */
BaseMotorController::~BaseMotorController() {
	delete _sensorColl;
	_sensorColl = 0;
}
/**
 * @return CCI handle for child classes.
 */
void* BaseMotorController::GetHandle() {
	return m_handle;
}
/**
 * Returns the Device ID
 *
 * @return Device number.
 */
int BaseMotorController::GetDeviceID() {
	int devID = 0;
	(void) c_MotController_GetDeviceNumber(m_handle, &devID);
	return devID;
}
//------ Set output routines. ----------//
/**
 * @param Mode Sets the appropriate output on the talon, depending on the mode.
 * @param value The output value to apply.
 *
 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
 * In Current mode, output value is in amperes.
 * In Velocity mode, output value is in position change / 100ms.
 * In Position mode, output value is in encoder ticks or an analog value,
 *   depending on the sensor.
 * In Follower mode, the output value is the integer device ID of the talon to
 * duplicate.
 *
 * @param value The setpoint value, as described above.
 *
 *
 *	Standard Driving Example:
 *	_talonLeft.set(ControlMode.PercentOutput, leftJoy);
 *	_talonRght.set(ControlMode.PercentOutput, rghtJoy);
 */
void BaseMotorController::Set(ControlMode Mode, double value) {
	Set(Mode, value, DemandType_Neutral, 0);
}
/**
 * @param mode Sets the appropriate output on the talon, depending on the mode.
 * @param demand0 The output value to apply.
 * 	such as advanced feed forward and/or auxiliary close-looping in firmware.
 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
 * In Current mode, output value is in amperes.
 * In Velocity mode, output value is in position change / 100ms.
 * In Position mode, output value is in encoder ticks or an analog value,
 *   depending on the sensor. See
 * In Follower mode, the output value is the integer device ID of the talon to
 * duplicate.
 *
 * @param demand1 Supplemental value.  This will also be control mode specific for future features.
 */

void BaseMotorController::Set(ControlMode mode, double demand0, double demand1) {
	Set(mode, demand0, DemandType_Neutral, demand1);
}
/**
 * @param mode Sets the appropriate output on the talon, depending on the mode.
 * @param demand0 The output value to apply.
 * 	such as advanced feed forward and/or auxiliary close-looping in firmware.
 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
 * In Current mode, output value is in amperes.
 * In Velocity mode, output value is in position change / 100ms.
 * In Position mode, output value is in encoder ticks or an analog value,
 *   depending on the sensor. See
 * In Follower mode, the output value is the integer device ID of the talon to
 * duplicate.
 *
 * @param demand1Type The demand type for demand1.
 * Neutral: Ignore demand1 and apply no change to the demand0 output.
 * AuxPID: Use demand1 to set the target for the auxiliary PID 1.
 * ArbitraryFeedForward: Use demand1 as an arbitrary additive value to the
 *	 demand0 output.  In PercentOutput the demand0 output is the motor output,
 *   and in closed-loop modes the demand0 output is the output of PID0.
 * @param demand1 Supplmental output value.  Units match the set mode.
 *
 *
 *  Arcade Drive Example:
 *		_talonLeft.set(ControlMode::PercentOutput, joyForward, DemandType_ArbitraryFeedForward, +joyTurn);
 *		_talonRght.set(ControlMode::PercentOutput, joyForward, DemandType_ArbitraryFeedForward, -joyTurn);
 *
 *	Drive Straight Example:
 *	Note: Selected Sensor Configuration is necessary for both PID0 and PID1.
 *		_talonLeft.follow(_talonRght, FollwerType_AuxOutput1);
 *		_talonRght.set(ControlMode::PercentOutput, joyForward, DemandType_AuxPID, desiredRobotHeading);
 *
 *	Drive Straight to a Distance Example:
 *	Note: Other configurations (sensor selection, PID gains, etc.) need to be set.
 *		_talonLeft.follow(_talonRght, FollwerType_AuxOutput1);
 *		_talonRght.set(ControlMode::MotionMagic, targetDistance, DemandType_AuxPID, desiredRobotHeading);
 */
void BaseMotorController::Set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
	m_controlMode = mode;
	m_sendMode = mode;
	m_setPoint = demand0;

	uint32_t work;
	switch (m_controlMode) {
	case ControlMode::PercentOutput:
		//case ControlMode::TimedPercentOutput:
		c_MotController_Set_4(m_handle, (int) m_sendMode, demand0, demand1, demand1Type);
		break;
	case ControlMode::Follower:
		/* did caller specify device ID */
		if ((0 <= demand0) && (demand0 <= 62)) { // [0,62]
			work = (uint32_t) GetBaseID();
			work >>= 16;
			work <<= 8;
			work |= (uint8_t) demand0;
		} else {
			work = (uint32_t) demand0;
		}
		/* single precision guarantees 16bits of integral precision,
		 *  so float/double cast on work is safe */
		c_MotController_Set_4(m_handle, (int) m_sendMode, (double)work, demand1, demand1Type);
		break;
	case ControlMode::Velocity:
	case ControlMode::Position:
	case ControlMode::MotionMagic:
	//case ControlMode::MotionMagicArc:
	case ControlMode::MotionProfile:
	case ControlMode::MotionProfileArc:
		c_MotController_Set_4(m_handle, (int) m_sendMode, demand0, demand1, demand1Type);
		break;
	case ControlMode::Current:
		c_MotController_SetDemand(m_handle, (int) m_sendMode,
				(int) (1000 * demand0), 0); /* milliamps */
		break;
	case ControlMode::Disabled:
		/* fall thru...*/
	default:
		c_MotController_SetDemand(m_handle, (int) m_sendMode, 0, 0);
		break;
	}
}
/**
 * Neutral the motor output by setting control mode to disabled.
 */
void BaseMotorController::NeutralOutput() {
	Set(ControlMode::Disabled, 0);
}
/**
 * Sets the mode of operation during neutral throttle output.
 *
 * @param neutralMode
 *            The desired mode of operation when the Controller output
 *            throttle is neutral (ie brake/coast)
 **/
void BaseMotorController::SetNeutralMode(NeutralMode neutralMode) {
	c_MotController_SetNeutralMode(m_handle, neutralMode);
}
/**
 * Enables a future feature called "Heading Hold".
 * For now this simply updates the CAN signal to the motor controller.
 * Future firmware updates will use this.
 *
 *	@param enable true/false enable
 */
void BaseMotorController::EnableHeadingHold(bool enable) {
	(void)enable;
	/* this routine is moot as the Set() call updates the signal on each call */
	//c_MotController_EnableHeadingHold(m_handle, enable);
}
/**
 * For now this simply updates the CAN signal to the motor controller.
 * Future firmware updates will use this to control advanced auxiliary loop behavior.
 *
 *	@param value
 */
void BaseMotorController::SelectDemandType(bool value) {
	(void)value;
	/* this routine is moot as the Set() call updates the signal on each call */
	//c_MotController_SelectDemandType(m_handle, value);
}

//------ Invert behavior ----------//
/**
 * Sets the phase of the sensor. Use when controller forward/reverse output
 * doesn't correlate to appropriate forward/reverse reading of sensor.
 * Pick a value so that positive PercentOutput yields a positive change in sensor.
 * After setting this, user can freely call SetInvert() with any value.
 *
 * @param PhaseSensor
 *            Indicates whether to invert the phase of the sensor.
 */
void BaseMotorController::SetSensorPhase(bool PhaseSensor) {
	c_MotController_SetSensorPhase(m_handle, PhaseSensor);
}

/**
 * Inverts the hbridge output of the motor controller.
 *
 * This does not impact sensor phase and should not be used to correct sensor polarity.
 *
 * This will invert the hbridge output but NOT the LEDs.
 * This ensures....
 *  - Green LEDs always represents positive request from robot-controller/closed-looping mode.
 *  - Green LEDs correlates to forward limit switch.
 *  - Green LEDs correlates to forward soft limit.
 *
 * @param invert
 *            Invert state to set.
 */
void BaseMotorController::SetInverted(bool invert) {
	_invert = invert; /* cache for getter */
	c_MotController_SetInverted(m_handle, _invert);
}
/**
 * @return invert setting of motor output.
 */
bool BaseMotorController::GetInverted() const {
	return _invert;
}

//----- general output shaping ------------------//
/**
 * Configures the open-loop ramp rate of throttle output.
 *
 * @param secondsFromNeutralToFull
 *            Minimum desired time to go from neutral to full throttle. A
 *            value of '0' will disable the ramp.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigOpenloopRamp(
		double secondsFromNeutralToFull, int timeoutMs) {
	return c_MotController_ConfigOpenLoopRamp(m_handle,
			secondsFromNeutralToFull, timeoutMs);
}

/**
 * Configures the closed-loop ramp rate of throttle output.
 *
 * @param secondsFromNeutralToFull
 *            Minimum desired time to go from neutral to full throttle. A
 *            value of '0' will disable the ramp.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigClosedloopRamp(
		double secondsFromNeutralToFull, int timeoutMs) {
	return c_MotController_ConfigClosedLoopRamp(m_handle,
			secondsFromNeutralToFull, timeoutMs);
}

/**
 * Configures the forward peak output percentage.
 *
 * @param percentOut
 *            Desired peak output percentage. [0,+1]
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigPeakOutputForward(double percentOut,
		int timeoutMs) {
	return c_MotController_ConfigPeakOutputForward(m_handle, percentOut,
			timeoutMs);
}

/**
 * Configures the reverse peak output percentage.
 *
 * @param percentOut
 *            Desired peak output percentage. [-1,0]
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigPeakOutputReverse(double percentOut,
		int timeoutMs) {
	return c_MotController_ConfigPeakOutputReverse(m_handle, percentOut,
			timeoutMs);
}
/**
 * Configures the forward nominal output percentage.
 *
 * @param percentOut
 *            Nominal (minimum) percent output. [0,+1]
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigNominalOutputForward(double percentOut,
		int timeoutMs) {
	return c_MotController_ConfigNominalOutputForward(m_handle, percentOut,
			timeoutMs);
}
/**
 * Configures the reverse nominal output percentage.
 *
 * @param percentOut
 *            Nominal (minimum) percent output. [-1,0]
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigNominalOutputReverse(double percentOut,
		int timeoutMs) {
	return c_MotController_ConfigNominalOutputReverse(m_handle, percentOut,
			timeoutMs);
}
/**
 * Configures the output deadband percentage.
 *
 * @param percentDeadband
 *            Desired deadband percentage. Minimum is 0.1%, Maximum is
 *            25%.  Pass 0.04 for 4% (factory default).
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigNeutralDeadband(double percentDeadband,
		int timeoutMs) {
	return c_MotController_ConfigNeutralDeadband(m_handle, percentDeadband,
			timeoutMs);
}

//------ Voltage Compensation ----------//
/**
 * Configures the Voltage Compensation saturation voltage.
 *
 * @param voltage
 *            This is the max voltage to apply to the hbridge when voltage
 *            compensation is enabled.  For example, if 10 (volts) is specified
 *            and a TalonSRX is commanded to 0.5 (PercentOutput, closed-loop, etc)
 *            then the TalonSRX will attempt to apply a duty-cycle to produce 5V.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigVoltageCompSaturation(double voltage,
		int timeoutMs) {
	return c_MotController_ConfigVoltageCompSaturation(m_handle, voltage,
			timeoutMs);
}

/**
 * Configures the voltage measurement filter.
 *
 * @param filterWindowSamples
 *            Number of samples in the rolling average of voltage
 *            measurement.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigVoltageMeasurementFilter(
		int filterWindowSamples, int timeoutMs) {
	return c_MotController_ConfigVoltageMeasurementFilter(m_handle,
			filterWindowSamples, timeoutMs);
}

/**
 * Enables voltage compensation. If enabled, voltage compensation works in
 * all control modes.
 *
 * @param enable
 *            Enable state of voltage compensation.
 **/
void BaseMotorController::EnableVoltageCompensation(bool enable) {
	c_MotController_EnableVoltageCompensation(m_handle, enable);
}

//------ General Status ----------//
/**
 * Gets the bus voltage seen by the device.
 *
 * @return The bus voltage value (in volts).
 */
double BaseMotorController::GetBusVoltage() {
	double param = 0;
	c_MotController_GetBusVoltage(m_handle, &param);
	return param;
}

/**
 * Gets the output percentage of the motor controller.
 *
 * @return Output of the motor controller (in percent).  [-1,+1]
 */
double BaseMotorController::GetMotorOutputPercent() {
	double param = 0;
	c_MotController_GetMotorOutputPercent(m_handle, &param);
	return param;
}

/**
 * @return applied voltage to motor in volts.
 */
double BaseMotorController::GetMotorOutputVoltage() {
	return GetBusVoltage() * GetMotorOutputPercent();
}

/**
 * Gets the output current of the motor controller.
 *
 * @return The output current (in amps).
 */
double BaseMotorController::GetOutputCurrent() {
	double param = 0;
	c_MotController_GetOutputCurrent(m_handle, &param);
	return param;
}
/**
 * Gets the temperature of the motor controller.
 *
 * @return Temperature of the motor controller (in 'C)
 */
double BaseMotorController::GetTemperature() {
	double param = 0;
	c_MotController_GetTemperature(m_handle, &param);
	return param;
}

//------ sensor selection ----------//
/**
 * Select the remote feedback device for the motor controller.
 * Most CTRE CAN motor controllers will support remote sensors over CAN.
 *
 * @param feedbackDevice
 *            Remote Feedback Device to select.
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigSelectedFeedbackSensor(
		RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
	return c_MotController_ConfigSelectedFeedbackSensor(m_handle,
			feedbackDevice, pidIdx, timeoutMs);
}
/**
 * Select the feedback device for the motor controller.
 *
 * @param feedbackDevice
 *            Feedback Device to select.
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigSelectedFeedbackSensor(
		FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) {
	return c_MotController_ConfigSelectedFeedbackSensor(m_handle,
			feedbackDevice, pidIdx, timeoutMs);
}

/**
 * The Feedback Coefficient is a scalar applied to the value of the
 * feedback sensor.  Useful when you need to scale your sensor values
 * within the closed-loop calculations.  Default value is 1.
 *
 * Selected Feedback Sensor register in firmware is the decoded sensor value
 * multiplied by the Feedback Coefficient.
 *
 * @param coefficient
 *            Feedback Coefficient value.  Maximum value of 1.
 *						Resolution is 1/(2^16).  Cannot be 0.
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigSelectedFeedbackCoefficient(
		double coefficient, int pidIdx, int timeoutMs) {
	return c_MotController_ConfigSelectedFeedbackCoefficient(m_handle,
			coefficient, pidIdx, timeoutMs);
}

/**
 * Select what remote device and signal to assign to Remote Sensor 0 or Remote Sensor 1.
 * After binding a remote device and signal to Remote Sensor X, you may select Remote Sensor X
 * as a PID source for closed-loop features.
 *
 * @param deviceID
 *            The CAN ID of the remote sensor device.
 * @param remoteSensorSource
 *            The remote sensor device and signal type to bind.
 * @param remoteOrdinal
 *            0 for configuring Remote Sensor 0
 *            1 for configuring Remote Sensor 1
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigRemoteFeedbackFilter(int deviceID,
		RemoteSensorSource remoteSensorSource, int remoteOrdinal,
		int timeoutMs) {
	return c_MotController_ConfigRemoteFeedbackFilter(m_handle, deviceID,
			(int) remoteSensorSource, remoteOrdinal, timeoutMs);
}
/**
 * Select what sensor term should be bound to switch feedback device.
 * Sensor Sum = Sensor Sum Term 0 - Sensor Sum Term 1
 * Sensor Difference = Sensor Diff Term 0 - Sensor Diff Term 1
 * The four terms are specified with this routine.  Then Sensor Sum/Difference
 * can be selected for closed-looping.
 *
 * @param sensorTerm Which sensor term to bind to a feedback source.
 * @param feedbackDevice The sensor signal to attach to sensorTerm.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigSensorTerm(SensorTerm sensorTerm,
		FeedbackDevice feedbackDevice, int timeoutMs) {
	return c_MotController_ConfigSensorTerm(m_handle, (int) sensorTerm,
			(int) feedbackDevice, timeoutMs);
}

//------- sensor status --------- //
/**
 * Get the selected sensor position (in raw sensor units).
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * See Phoenix-Documentation for how to interpret.
 *
 * @return Position of selected sensor (in raw sensor units).
 */
int BaseMotorController::GetSelectedSensorPosition(int pidIdx) {
	int retval;
	c_MotController_GetSelectedSensorPosition(m_handle, &retval, pidIdx);
	return retval;
}
/**
 * Get the selected sensor velocity.
 *
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * @return selected sensor (in raw sensor units) per 100ms.
 * See Phoenix-Documentation for how to interpret.
 */
int BaseMotorController::GetSelectedSensorVelocity(int pidIdx) {
	int retval;
	c_MotController_GetSelectedSensorVelocity(m_handle, &retval, pidIdx);
	return retval;
}
/**
 * Sets the sensor position to the given value.
 *
 * @param sensorPos
 *            Position to set for the selected sensor (in raw sensor units).
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::SetSelectedSensorPosition(int sensorPos,
		int pidIdx, int timeoutMs) {
	return c_MotController_SetSelectedSensorPosition(m_handle, sensorPos,
			pidIdx, timeoutMs);
}

//------ status frame period changes ----------//
/**
 * Sets the period of the given control frame.
 *
 * @param frame
 *            Frame whose period is to be changed.
 * @param periodMs
 *            Period in ms for the given frame.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::SetControlFramePeriod(ControlFrame frame,
		int periodMs) {
	return c_MotController_SetControlFramePeriod(m_handle, frame, periodMs);
}
/**
 * Sets the period of the given status frame.
 *
 * User ensure CAN Bus utilization is not high.
 *
 * This setting is not persistent and is lost when device is reset.
 * If this is a concern, calling application can use HasReset()
 * to determine if the status frame needs to be reconfigured.
 *
 * @param frame
 *            Frame whose period is to be changed.
 * @param periodMs
 *            Period in ms for the given frame.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::SetStatusFramePeriod(StatusFrame frame,
		int periodMs, int timeoutMs) {
	return c_MotController_SetStatusFramePeriod(m_handle, frame, periodMs,
			timeoutMs);
}
/**
 * Sets the period of the given status frame.
 *
 * User ensure CAN Bus utilization is not high.
 *
 * This setting is not persistent and is lost when device is reset.
 * If this is a concern, calling application can use HasReset()
 * to determine if the status frame needs to be reconfigured.
 *
 * @param frame
 *            Frame whose period is to be changed.
 * @param periodMs
 *            Period in ms for the given frame.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::SetStatusFramePeriod(StatusFrameEnhanced frame,
		int periodMs, int timeoutMs) {
	return c_MotController_SetStatusFramePeriod(m_handle, frame, periodMs,
			timeoutMs);
}
/**
 * Gets the period of the given status frame.
 *
 * @param frame
 *            Frame to get the period of.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Period of the given status frame.
 */
int BaseMotorController::GetStatusFramePeriod(StatusFrame frame,
		int timeoutMs) {
	int periodMs = 0;
	c_MotController_GetStatusFramePeriod(m_handle, frame, &periodMs, timeoutMs);
	return periodMs;
}

/**
 * Gets the period of the given status frame.
 *
 * @param frame
 *            Frame to get the period of.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Period of the given status frame.
 */
int BaseMotorController::GetStatusFramePeriod(StatusFrameEnhanced frame,
		int timeoutMs) {
	int periodMs = 0;
	c_MotController_GetStatusFramePeriod(m_handle, frame, &periodMs, timeoutMs);
	return periodMs;
}

//----- velocity signal conditioning ------//

/**
 * Configures the period of each velocity sample.
 * Every 1ms a position value is sampled, and the delta between that sample
 * and the position sampled kPeriod ms ago is inserted into a filter.
 * kPeriod is configured with this function.
 *
 * @param period
 *            Desired period for the velocity measurement. @see
 *            #VelocityMeasPeriod
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigVelocityMeasurementPeriod(
		VelocityMeasPeriod period, int timeoutMs) {
	return c_MotController_ConfigVelocityMeasurementPeriod(m_handle, period,
			timeoutMs);
}
/**
 * Sets the number of velocity samples used in the rolling average velocity
 * measurement.
 *
 * @param windowSize
 *            Number of samples in the rolling average of velocity
 *            measurement. Valid values are 1,2,4,8,16,32. If another
 *            value is specified, it will truncate to nearest support value.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigVelocityMeasurementWindow(int windowSize,
		int timeoutMs) {
	return c_MotController_ConfigVelocityMeasurementWindow(m_handle, windowSize,
			timeoutMs);
}

//------ remote limit switch ----------//
/**
 * Configures the forward limit switch for a remote source.
 * For example, a CAN motor controller may need to monitor the Limit-F pin
 * of another Talon or CANifier.
 *
 * @param type
 *            Remote limit switch source.
 *            User can choose between a remote Talon SRX, CANifier, or deactivate the feature.
 * @param normalOpenOrClose
 *            Setting for normally open, normally closed, or disabled. This setting
 *            matches the web-based configuration drop down.
 * @param deviceID
 *            Device ID of remote source (Talon SRX or CANifier device ID).
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigForwardLimitSwitchSource(
		RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
		int deviceID, int timeoutMs) {
	LimitSwitchSource cciType = LimitSwitchRoutines::Promote(type);
	return c_MotController_ConfigForwardLimitSwitchSource(m_handle, cciType,
			normalOpenOrClose, deviceID, timeoutMs);
}
/**
 * Configures the reverse limit switch for a remote source.
 * For example, a CAN motor controller may need to monitor the Limit-R pin
 * of another Talon or CANifier.
 *
 * @param type
 *            Remote limit switch source.
 *            User can choose between a remote Talon SRX, CANifier, or deactivate the feature.
 * @param normalOpenOrClose
 *            Setting for normally open, normally closed, or disabled. This setting
 *            matches the web-based configuration drop down.
 * @param deviceID
 *            Device ID of remote source (Talon SRX or CANifier device ID).
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigReverseLimitSwitchSource(
		RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
		int deviceID, int timeoutMs) {
	LimitSwitchSource cciType = LimitSwitchRoutines::Promote(type);
	return c_MotController_ConfigReverseLimitSwitchSource(m_handle, cciType,
			normalOpenOrClose, deviceID, timeoutMs);
}
/**
 * Sets the enable state for limit switches.
 *
 * This routine can be used to DISABLE the limit switch feature.
 * This is helpful to force off the limit switch detection.
 * For example, a module can leave limit switches enable for home-ing
 * a continuous mechanism, and once done this routine can force off
 * disabling of the motor controller.
 *
 * Limit switches must be enabled using the Config routines first.
 *
 * @param enable
 *            Enable state for limit switches.
 */
void BaseMotorController::OverrideLimitSwitchesEnable(bool enable) {
	c_MotController_OverrideLimitSwitchesEnable(m_handle, enable);
}

//------ local limit switch ----------//
/**
 * Configures a limit switch for a local/remote source.
 *
 * For example, a CAN motor controller may need to monitor the Limit-R pin
 * of another Talon, CANifier, or local Gadgeteer feedback connector.
 *
 * If the sensor is remote, a device ID of zero is assumed.
 * If that's not desired, use the four parameter version of this function.
 *
 * @param type
 *            Limit switch source. @see #LimitSwitchSource
 *            User can choose between the feedback connector, remote Talon SRX, CANifier, or deactivate the feature.
 * @param normalOpenOrClose
 *            Setting for normally open, normally closed, or disabled. This setting
 *            matches the web-based configuration drop down.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigForwardLimitSwitchSource(
		LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
		int timeoutMs) {
	return c_MotController_ConfigForwardLimitSwitchSource(m_handle, type,
			normalOpenOrClose, 0, timeoutMs);
}
/**
 * Configures a limit switch for a local/remote source.
 *
 * For example, a CAN motor controller may need to monitor the Limit-R pin
 * of another Talon, CANifier, or local Gadgeteer feedback connector.
 *
 * If the sensor is remote, a device ID of zero is assumed.
 * If that's not desired, use the four parameter version of this function.
 *
 * @param type
 *            Limit switch source. @see #LimitSwitchSource
 *            User can choose between the feedback connector, remote Talon SRX, CANifier, or deactivate the feature.
 * @param normalOpenOrClose
 *            Setting for normally open, normally closed, or disabled. This setting
 *            matches the web-based configuration drop down.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigReverseLimitSwitchSource(
		LimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
		int timeoutMs) {
	return c_MotController_ConfigReverseLimitSwitchSource(m_handle, type,
			normalOpenOrClose, 0, timeoutMs);
}

//------ soft limit ----------//
/**
 * Configures the forward soft limit threshold.
 *
 * @param forwardSensorLimit
 *            Forward Sensor Position Limit (in raw sensor units).
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigForwardSoftLimitThreshold(int forwardSensorLimit,
		int timeoutMs) {
	return c_MotController_ConfigForwardSoftLimitThreshold(m_handle, forwardSensorLimit,
			timeoutMs);
}

/**
 * Configures the reverse soft limit threshold.
 *
 * @param reverseSensorLimit
 *            Reverse Sensor Position Limit (in raw sensor units).
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigReverseSoftLimitThreshold(int reverseSensorLimit,
		int timeoutMs) {
	return c_MotController_ConfigReverseSoftLimitThreshold(m_handle, reverseSensorLimit,
			timeoutMs);
}

/**
 * Configures the forward soft limit enable .
 *
 * @param enable
 *            True to enable soft limit. False to disable.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigForwardSoftLimitEnable(bool enable,
		int timeoutMs) {
	return c_MotController_ConfigForwardSoftLimitEnable(m_handle, enable,
			timeoutMs);
}


/**
 * Configures the reverse soft limit enable.
 *
 * @param enable
 *            True to enable soft limit. False to disable.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigReverseSoftLimitEnable(bool enable,
		int timeoutMs) {
	return c_MotController_ConfigReverseSoftLimitEnable(m_handle, enable,
			timeoutMs);
}

/**
 * Can be used to override-disable the soft limits.
 * This function can be used to quickly disable soft limits without
 * having to modify the persistent configuration.
 *
 * @param enable
 *            Enable state for soft limit switches.
 **/
void BaseMotorController::OverrideSoftLimitsEnable(bool enable) {
	c_MotController_OverrideSoftLimitsEnable(m_handle, enable);
}

//------ Current Lim ----------//
/* not available in base */

//------ General Close loop ----------//
/**
 * Sets the 'P' constant in the given parameter slot.
 *
 * @param slotIdx
 *            Parameter slot for the constant.
 * @param value
 *            Value of the P constant.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::Config_kP(int slotIdx, double value,
		int timeoutMs) {
	return c_MotController_Config_kP(m_handle, slotIdx, value, timeoutMs);
}

/**
 * Sets the 'I' constant in the given parameter slot.
 *
 * @param slotIdx
 *            Parameter slot for the constant.
 * @param value
 *            Value of the I constant.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::Config_kI(int slotIdx, double value,
		int timeoutMs) {
	return c_MotController_Config_kI(m_handle, slotIdx, value, timeoutMs);
}

/**
 * Sets the 'D' constant in the given parameter slot.
 *
 * @param slotIdx
 *            Parameter slot for the constant.
 * @param value
 *            Value of the D constant.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::Config_kD(int slotIdx, double value,
		int timeoutMs) {
	return c_MotController_Config_kD(m_handle, slotIdx, value, timeoutMs);
}

/**
 * Sets the 'F' constant in the given parameter slot.
 *
 * @param slotIdx
 *            Parameter slot for the constant.
 * @param value
 *            Value of the F constant.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::Config_kF(int slotIdx, double value,
		int timeoutMs) {
	return c_MotController_Config_kF(m_handle, slotIdx, value, timeoutMs);
}

/**
 * Sets the Integral Zone constant in the given parameter slot.
 * If the (absolute) closed-loop error is outside of this zone, integral accumulator
 * is automatically cleared.  This ensures than integral wind up events will stop after
 * the sensor gets far enough from its target.
 *
 * @param slotIdx
 *            Parameter slot for the constant.
 * @param izone
 *            Value of the Integral Zone constant (closed loop error units X 1ms).
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::Config_IntegralZone(int slotIdx, int izone,
		int timeoutMs) {
	return c_MotController_Config_IntegralZone(m_handle, slotIdx, izone,
			timeoutMs);
}

/**
 * Sets the allowable closed-loop error in the given parameter slot.
 * If (absolute) closed-loop error is within this value, the motor output is neutral.
 *
 * @param slotIdx
 *            Parameter slot for the constant.
 * @param allowableCloseLoopError
 *            Value of the allowable closed-loop error.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigAllowableClosedloopError(int slotIdx,
		int allowableCloseLoopError, int timeoutMs) {
	return c_MotController_ConfigAllowableClosedloopError(m_handle, slotIdx,
			allowableCloseLoopError, timeoutMs);
}

/**
 * Sets the maximum integral accumulator in the given parameter slot.
 *
 * @param slotIdx
 *            Parameter slot for the constant.
 * @param iaccum
 *            Value of the maximum integral accumulator (closed loop error units X 1ms).
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigMaxIntegralAccumulator(int slotIdx,
		double iaccum, int timeoutMs) {
	return c_MotController_ConfigMaxIntegralAccumulator(m_handle, slotIdx,
			iaccum, timeoutMs);
}

/**
 * Sets the peak closed-loop output.  This peak output is slot-specific and
 *   is applied to the output of the associated PID loop.
 * This setting is seperate from the generic Peak Output setting.
 *
 * @param slotIdx
 *            Parameter slot for the constant.
 * @param percentOut
 *            Peak Percent Output from 0 to 1.  This value is absolute and
 *						the magnitude will apply in both forward and reverse directions.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs) {
	return c_MotController_ConfigClosedLoopPeakOutput(m_handle, slotIdx, percentOut, timeoutMs);
}

/**
 * Sets the loop time (in milliseconds) of the PID closed-loop calculations.
 * Default value is 1 ms.
 *
 * @param slotIdx
 *            Parameter slot for the constant.
 * @param loopTimeMs
 *            Loop timing of the closed-loop calculations.  Minimum value of
 *						1 ms, maximum of 64 ms.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs) {
	return c_MotController_ConfigClosedLoopPeriod(m_handle, slotIdx, loopTimeMs, timeoutMs);
}

/**
	 * Configures the Polarity of the Auxiliary PID (PID1).
	 *
	 * Standard Polarity:
	 *    Primary Output = PID0 + PID1
	 *    Auxiliary Output = PID0 - PID1
	 *
	 * Inverted Polarity:
	 *    Primary Output = PID0 - PID1
	 *    Auxiliary Output = PID0 + PID1
	 *
	 * @param invert
	 *            If true, use inverted PID1 output polarity.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code
	 */
	ErrorCode BaseMotorController::ConfigAuxPIDPolarity(bool invert, int timeoutMs){
		return ConfigSetParameter(ParamEnum::ePIDLoopPolarity, invert, 0, 1, timeoutMs);
	}

/**
 * Sets the integral accumulator. Typically this is used to clear/zero
 * the integral accumulator, however some use cases may require seeding
 * the accumulator for a faster response.
 *
 * @param iaccum
 *            Value to set for the integral accumulator (closed loop error units X 1ms).
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::SetIntegralAccumulator(double iaccum, int pidIdx,
		int timeoutMs) {
	return c_MotController_SetIntegralAccumulator(m_handle, iaccum, pidIdx,
			timeoutMs);
}

/**
 * Gets the closed-loop error.
 * The units depend on which control mode is in use.
 * See Phoenix-Documentation information on units.
 *
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * @return Closed-loop error value.
 */
int BaseMotorController::GetClosedLoopError(int pidIdx) {
	int closedLoopError = 0;
	c_MotController_GetClosedLoopError(m_handle, &closedLoopError, pidIdx);
	return closedLoopError;
}

/**
 * Gets the iaccum value.
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * @return Integral accumulator value (Closed-loop error X 1ms).
 */
double BaseMotorController::GetIntegralAccumulator(int pidIdx) {
	double iaccum = 0;
	c_MotController_GetIntegralAccumulator(m_handle, &iaccum, pidIdx);
	return iaccum;
}


/**
 * Gets the derivative of the closed-loop error.
 *
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * @return The error derivative value.
 */
double BaseMotorController::GetErrorDerivative(int pidIdx) {
	double derror = 0;
	c_MotController_GetErrorDerivative(m_handle, &derror, pidIdx);
	return derror;
}

/**
 * Selects which profile slot to use for closed-loop control.
 *
 * @param slotIdx
 *            Profile slot to select.
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 **/
ErrorCode BaseMotorController::SelectProfileSlot(int slotIdx, int pidIdx) {
	return c_MotController_SelectProfileSlot(m_handle, slotIdx, pidIdx);
}
/**
 * Gets the current target of a given closed loop.
 *
 * @param pidIdx
 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
 * @return The closed loop target.
 */
int BaseMotorController::GetClosedLoopTarget(int pidIdx) {
	int param = 0;
	c_MotController_GetClosedLoopTarget(m_handle, &param, pidIdx);
	return param;
}
/**
 * Gets the active trajectory target position using
 * MotionMagic/MotionProfile control modes.
 *
 * @return The Active Trajectory Position in sensor units.
 */
int BaseMotorController::GetActiveTrajectoryPosition() {
	int param = 0;
	c_MotController_GetActiveTrajectoryPosition(m_handle, &param);
	return param;
}
/**
 * Gets the active trajectory target velocity using
 * MotionMagic/MotionProfile control modes.
 *
 * @return The Active Trajectory Velocity in sensor units per 100ms.
 */
int BaseMotorController::GetActiveTrajectoryVelocity() {
	int param = 0;
	c_MotController_GetActiveTrajectoryVelocity(m_handle, &param);
	return param;
}
/**
 * Gets the active trajectory target heading using
 * MotionMagicArc/MotionProfileArc control modes.
 *
 * @return The Active Trajectory Heading in degreees.
 */
double BaseMotorController::GetActiveTrajectoryHeading() {
	double param = 0;
	c_MotController_GetActiveTrajectoryHeading(m_handle, &param);
	return param;
}

//------ Motion Profile Settings used in Motion Magic and Motion Profile ----------//

/**
 * Sets the Motion Magic Cruise Velocity.  This is the peak target velocity
 * that the motion magic curve generator can use.
 *
 * @param sensorUnitsPer100ms
 *            Motion Magic Cruise Velocity (in raw sensor units per 100 ms).
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigMotionCruiseVelocity(
		int sensorUnitsPer100ms, int timeoutMs) {
	return c_MotController_ConfigMotionCruiseVelocity(m_handle,
			sensorUnitsPer100ms, timeoutMs);
}
/**
 * Sets the Motion Magic Acceleration.  This is the target acceleration
 * that the motion magic curve generator can use.
 *
 * @param sensorUnitsPer100msPerSec
 *            Motion Magic Acceleration (in raw sensor units per 100 ms per
 *            second).
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigMotionAcceleration(
		int sensorUnitsPer100msPerSec, int timeoutMs) {
	return c_MotController_ConfigMotionAcceleration(m_handle,
			sensorUnitsPer100msPerSec, timeoutMs);
}

//------ Motion Profile Buffer ----------//
/**
 * Clear the buffered motion profile in both motor controller's RAM (bottom), and in the API
 * (top).
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ClearMotionProfileTrajectories() {
	return c_MotController_ClearMotionProfileTrajectories(m_handle);
}
/**
 * Retrieve just the buffer count for the api-level (top) buffer.
 * This routine performs no CAN or data structure lookups, so its fast and ideal
 * if caller needs to quickly poll the progress of trajectory points being
 * emptied into motor controller's RAM. Otherwise just use GetMotionProfileStatus.
 * @return number of trajectory points in the top buffer.
 */
int BaseMotorController::GetMotionProfileTopLevelBufferCount() {
	int param = 0;
	c_MotController_GetMotionProfileTopLevelBufferCount(m_handle, &param);
	return param;
}
/**
 * Push another trajectory point into the top level buffer (which is emptied
 * into the motor controller's bottom buffer as room allows).
 * @param trajPt to push into buffer.
 * The members should be filled in with these values...
 *
 * 		targPos:  servo position in sensor units.
 *		targVel:  velocity to feed-forward in sensor units
 *                 per 100ms.
 * 		profileSlotSelect0  Which slot to get PIDF gains. PID is used for position servo. F is used
 *						   as the Kv constant for velocity feed-forward. Typically this is hardcoded
 *						   to the a particular slot, but you are free gain schedule if need be.
 *						   Choose from [0,3]
 *		profileSlotSelect1 Which slot to get PIDF gains for auxiliary PId.
 *						   This only has impact during MotionProfileArc Control mode.
 *						   Choose from [0,1].
 * 	   isLastPoint  set to nonzero to signal motor controller to keep processing this
 *                     trajectory point, instead of jumping to the next one
 *                     when timeDurMs expires.  Otherwise MP executer will
 *                     eventually see an empty buffer after the last point
 *                     expires, causing it to assert the IsUnderRun flag.
 *                     However this may be desired if calling application
 *                     never wants to terminate the MP.
 *		zeroPos  set to nonzero to signal motor controller to "zero" the selected
 *                 position sensor before executing this trajectory point.
 *                 Typically the first point should have this set only thus
 *                 allowing the remainder of the MP positions to be relative to
 *                 zero.
 *		timeDur Duration to apply this trajectory pt.
 * 				This time unit is ADDED to the exising base time set by
 * 				configMotionProfileTrajectoryPeriod().
 * @return CTR_OKAY if trajectory point push ok. ErrorCode if buffer is
 *         full due to kMotionProfileTopBufferCapacity.
 */
ErrorCode BaseMotorController::PushMotionProfileTrajectory(
		const ctre::phoenix::motion::TrajectoryPoint & trajPt) {
	ErrorCode retval = c_MotController_PushMotionProfileTrajectory_2(m_handle,
			trajPt.position, trajPt.velocity, trajPt.auxiliaryPos,
			trajPt.profileSlotSelect0, trajPt.profileSlotSelect1, trajPt.isLastPoint, trajPt.zeroPos,
			(int)trajPt.timeDur);
	return retval;
}
/**
 * Retrieve just the buffer full for the api-level (top) buffer.
 * This routine performs no CAN or data structure lookups, so its fast and ideal
 * if caller needs to quickly poll. Otherwise just use GetMotionProfileStatus.
 * @return number of trajectory points in the top buffer.
 */
bool BaseMotorController::IsMotionProfileTopLevelBufferFull() {
	bool retval = false;
	c_MotController_IsMotionProfileTopLevelBufferFull(m_handle, &retval);
	return retval;
}
/**
 * This must be called periodically to funnel the trajectory points from the
 * API's top level buffer to the controller's bottom level buffer.  Recommendation
 * is to call this twice as fast as the execution rate of the motion profile.
 * So if MP is running with 20ms trajectory points, try calling this routine
 * every 10ms.  All motion profile functions are thread-safe through the use of
 * a mutex, so there is no harm in having the caller utilize threading.
 */
void BaseMotorController::ProcessMotionProfileBuffer() {
	c_MotController_ProcessMotionProfileBuffer(m_handle);
}
/**
 * Retrieve all status information.
 * For best performance, Caller can snapshot all status information regarding the
 * motion profile executer.
 *
 * @param [out] statusToFill  Caller supplied object to fill.
 *
 * The members are filled, as follows...
 *
 *	topBufferRem:	The available empty slots in the trajectory buffer.
 * 	 				The robot API holds a "top buffer" of trajectory points, so your applicaion
 * 	 				can dump several points at once.  The API will then stream them into the
 * 	 		 		low-level buffer, allowing the motor controller to act on them.
 *
 *	topBufferRem: The number of points in the top trajectory buffer.
 *
 *	btmBufferCnt: The number of points in the low level controller buffer.
 *
 *	hasUnderrun: 	Set if isUnderrun ever gets set.
 * 	 	 	 	 	Only is cleared by clearMotionProfileHasUnderrun() to ensure
 *
 *	isUnderrun:		This is set if controller needs to shift a point from its buffer into
 *					the active trajectory point however
 *					the buffer is empty.
 *					This gets cleared automatically when is resolved.
 *
 *	activePointValid:	True if the active trajectory point has not empty, false otherwise. The members in activePoint are only valid if this signal is set.
 *
 *	isLast:	is set/cleared based on the MP executer's current
 *                trajectory point's IsLast value.  This assumes
 *                IsLast was set when PushMotionProfileTrajectory
 *                was used to insert the currently processed trajectory
 *                point.
 *
 *	profileSlotSelect0: The currently processed trajectory point's
 *      			  selected slot.  This can differ in the currently selected slot used
 *       				 for Position and Velocity servo modes.   Must be within  [0,3].
*
 *	profileSlotSelect1: The currently processed trajectory point's
 *      			  selected slot for auxiliary PID.  This can differ in the currently selected slot used
 *       				 for Position and Velocity servo modes.  Must be within  [0,1].
 *
 *	outputEnable:		The current output mode of the motion profile
 *						executer (disabled, enabled, or hold).  When changing the set()
 *						value in MP mode, it's important to check this signal to
 *						confirm the change takes effect before interacting with the top buffer.
 */
ErrorCode BaseMotorController::GetMotionProfileStatus(
		ctre::phoenix::motion::MotionProfileStatus & statusToFill) {

	int outputEnable = 0;
	ErrorCode retval = c_MotController_GetMotionProfileStatus_2(m_handle,
			&statusToFill.topBufferRem, &statusToFill.topBufferCnt,
			&statusToFill.btmBufferCnt, &statusToFill.hasUnderrun,
			&statusToFill.isUnderrun, &statusToFill.activePointValid,
			&statusToFill.isLast, &statusToFill.profileSlotSelect0,
			&outputEnable, &statusToFill.timeDurMs, &statusToFill.profileSlotSelect1);

	statusToFill.outputEnable =
			(ctre::phoenix::motion::SetValueMotionProfile) outputEnable;

	return retval;
}
/**
 * Clear the "Has Underrun" flag.  Typically this is called after application
 * has confirmed an underrun had occured.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ClearMotionProfileHasUnderrun(int timeoutMs) {
	return c_MotController_ClearMotionProfileHasUnderrun(m_handle, timeoutMs);
}
/**
 * Calling application can opt to speed up the handshaking between the robot API
 * and the controller to increase the download rate of the controller's Motion Profile.
 * Ideally the period should be no more than half the period of a trajectory
 * point.
 * @param periodMs The transmit period in ms.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ChangeMotionControlFramePeriod(int periodMs) {
	return c_MotController_ChangeMotionControlFramePeriod(m_handle, periodMs);
}
/**
 * When trajectory points are processed in the motion profile executer, the MPE determines
 * how long to apply the active trajectory point by summing baseTrajDurationMs with the
 * timeDur of the trajectory point (see TrajectoryPoint).
 *
 * This allows general selection of the execution rate of the points with 1ms resolution,
 * while allowing some degree of change from point to point.
 * @param baseTrajDurationMs The base duration time of every trajectory point.
 * 							This is summed with the trajectory points unique timeDur.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs) {
	return c_MotController_ConfigMotionProfileTrajectoryPeriod(m_handle, baseTrajDurationMs, timeoutMs);
}

//------ error ----------//
/**
 * Gets the last error generated by this object.
 * Not all functions return an error code but can potentially report errors.
 * This function can be used to retrieve those error codes.
 *
 * @return Last Error Code generated by a function.
 */
ErrorCode BaseMotorController::GetLastError() {
	return c_MotController_GetLastError(m_handle);
}

//------ Faults ----------//
/**
 * Polls the various fault flags.
 * @param toFill Caller's object to fill with latest fault flags.
 * @return Last Error Code generated by a function.
 */
ErrorCode BaseMotorController::GetFaults(Faults & toFill) {
	int faultBits;
	ErrorCode retval = c_MotController_GetFaults(m_handle, &faultBits);
	toFill = Faults(faultBits);
	return retval;
}
/**
 * Polls the various sticky fault flags.
 * @param toFill Caller's object to fill with latest sticky fault flags.
 * @return Last Error Code generated by a function.
 */
ErrorCode BaseMotorController::GetStickyFaults(StickyFaults & toFill) {
	int faultBits;
	ErrorCode retval = c_MotController_GetStickyFaults(m_handle, &faultBits);
	toFill = StickyFaults(faultBits);
	return retval;
}
/**
 * Clears all sticky faults.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Last Error Code generated by a function.
 */
ErrorCode BaseMotorController::ClearStickyFaults(int timeoutMs) {
	return c_MotController_ClearStickyFaults(m_handle, timeoutMs);
}

//------ Firmware ----------//
/**
 * Gets the firmware version of the device.
 *
 * @return Firmware version of device.  For example: version 1-dot-2 is 0x0102.
 */
int BaseMotorController::GetFirmwareVersion() {
	int retval = -1;
	c_MotController_GetFirmwareVersion(m_handle, &retval);
	return retval;
}
/**
 * Returns true if the device has reset since last call.
 *
 * @return Has a Device Reset Occurred?
 */
bool BaseMotorController::HasResetOccurred() {
	bool retval = false;
	c_MotController_HasResetOccurred(m_handle, &retval);
	return retval;
}

//------ Custom Persistent Params ----------//
/**
 * Sets the value of a custom parameter. This is for arbitrary use.
 *
 * Sometimes it is necessary to save calibration/limit/target
 * information in the device. Particularly if the
 * device is part of a subsystem that can be replaced.
 *
 * @param newValue
 *            Value for custom parameter.
 * @param paramIndex
 *            Index of custom parameter [0,1]
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigSetCustomParam(int newValue,
		int paramIndex, int timeoutMs) {
	return c_MotController_ConfigSetCustomParam(m_handle, newValue, paramIndex,
			timeoutMs);
}

/**
 * Gets the value of a custom parameter.
 *
 * @param paramIndex
 *            Index of custom parameter [0,1].
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Value of the custom param.
 */
int BaseMotorController::ConfigGetCustomParam(int paramIndex, int timeoutMs) {
	int readValue;
	c_MotController_ConfigGetCustomParam(m_handle, &readValue, paramIndex,
			timeoutMs);
	return readValue;
}

//------ Generic Param API  ----------//
/**
 * Sets a parameter. Generally this is not used.
 * This can be utilized in
 * - Using new features without updating API installation.
 * - Errata workarounds to circumvent API implementation.
 * - Allows for rapid testing / unit testing of firmware.
 *
 * @param param
 *            Parameter enumeration.
 * @param value
 *            Value of parameter.
 * @param subValue
 *            Subvalue for parameter. Maximum value of 255.
 * @param ordinal
 *            Ordinal of parameter.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Error Code generated by function. 0 indicates no error.
 */
ErrorCode BaseMotorController::ConfigSetParameter(ParamEnum param, double value,
		uint8_t subValue, int ordinal, int timeoutMs) {
	return c_MotController_ConfigSetParameter(m_handle, param, value, subValue,
			ordinal, timeoutMs);
}

/**
 * Gets a parameter.
 *
 * @param param
 *            Parameter enumeration.
 * @param ordinal
 *            Ordinal of parameter.
 * @param timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
 * @return Value of parameter.
 */
double BaseMotorController::ConfigGetParameter(ParamEnum param, int ordinal,
		int timeoutMs) {
	double value = 0;
	c_MotController_ConfigGetParameter(m_handle, param, &value, ordinal,
			timeoutMs);
	return (double) value;
}

//------ Misc. ----------//
int BaseMotorController::GetBaseID() {
	return _arbId;
}
/**
 * @return control mode motor controller is in
 */
ControlMode BaseMotorController::GetControlMode() {
	return m_controlMode;
}

// ----- Follower ------//
/**
 * Set the control mode and output value so that this motor controller will
 * follow another motor controller. Currently supports following Victor SPX
 * and Talon SRX.
 *
 * @param masterToFollow
 *						Motor Controller object to follow.
 * @param followerType
 *						Type of following control.  Use AuxOutput1 to follow the master
 *						device's auxiliary output 1.
 *						Use PercentOutput for standard follower mode.
 */
void BaseMotorController::Follow(IMotorController & masterToFollow, FollowerType followerType) {
	uint32_t baseId = masterToFollow.GetBaseID();
	uint32_t id24 = (uint16_t) (baseId >> 0x10);
	id24 <<= 8;
	id24 |= (uint8_t) (baseId);

	switch (followerType) {
		case FollowerType_PercentOutput:
			Set(ControlMode::Follower, (double) id24);
			break;
		case FollowerType_AuxOutput1:
			/* follow the motor controller, but set the aux flag
			 * to ensure we follow the processed output */
			Set(ControlMode::Follower, (double) id24, DemandType_AuxPID, 0);
			break;
		default:
			NeutralOutput();
			break;
	}
}
/**
 * Set the control mode and output value so that this motor controller will
 * follow another motor controller.
 * Currently supports following Victor SPX and Talon SRX.
 */
void BaseMotorController::Follow(IMotorController & masterToFollow) {
	Follow(masterToFollow, FollowerType_PercentOutput);
}
/** When master makes a device, this routine is called to signal the update. */
void BaseMotorController::ValueUpdated() {
	//do nothing
}
/**
 * @return object that can get/set individual raw sensor values.
 */
ctre::phoenix::motorcontrol::SensorCollection & BaseMotorController::GetSensorCollection() {
	return *_sensorColl;
}
