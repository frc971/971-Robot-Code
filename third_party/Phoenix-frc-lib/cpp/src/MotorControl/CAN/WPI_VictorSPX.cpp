/**
 * WPI Compliant motor controller class.
 * WPILIB's object model requires many interfaces to be implemented to use
 * the various features.
 * This includes...
 * - Software PID loops running in the robot controller
 * - LiveWindow/Test mode features
 * - Motor Safety (auto-turn off of motor if Set stops getting called)
 * - Single Parameter set that assumes a simple motor controller.
 */
#include "ctre/phoenix/MotorControl/CAN/WPI_VictorSPX.h"
#include "ctre/phoenix/MotorControl/CAN/BaseMotorController.h"
#include "HAL/HAL.h"
#include <sstream>

using namespace ctre::phoenix::motorcontrol::can;


/** Constructor */
WPI_VictorSPX::WPI_VictorSPX(int deviceNumber) :
		BaseMotorController(deviceNumber | 0x01040000),
		VictorSPX(deviceNumber),
		_safetyHelper(this) {
	HAL_Report(HALUsageReporting::kResourceType_CTRE_future3, deviceNumber + 1);
	/* build string description */
	std::stringstream work;
	work << "Victor SPX " << deviceNumber;
	_desc = work.str();
	/* prep motor safety */
	_safetyHelper.SetExpiration(0.0);
	_safetyHelper.SetSafetyEnabled(false);
	SetName("Victor SPX ", deviceNumber);
}
WPI_VictorSPX::~WPI_VictorSPX() {
	/* MT */
}

//----------------------- set/get routines for WPILIB interfaces -------------------//
/**
 * Common interface for setting the speed of a simple speed controller.
 *
 * @param speed The speed to set.  Value should be between -1.0 and 1.0.
 * 									Value is also saved for Get().
 */
void WPI_VictorSPX::Set(double speed) {
	_speed = speed;
	VictorSPX::Set(ControlMode::PercentOutput, speed);
	_safetyHelper.Feed();
}
void WPI_VictorSPX::PIDWrite(double output) {
	Set(output);
}

/**
 * Common interface for getting the current set speed of a speed controller.
 *
 * @return The current set speed.  Value is between -1.0 and 1.0.
 */
double WPI_VictorSPX::Get() const {
	return _speed;
}

//----------------------- Intercept CTRE calls for motor safety -------------------//
void WPI_VictorSPX::Set(ControlMode mode, double value) {
	/* intercept the advanced Set and feed motor-safety */
	VictorSPX::Set(mode, value);
	_safetyHelper.Feed();
}
void WPI_VictorSPX::Set(ControlMode mode, double demand0, double demand1) {
	/* intercept the advanced Set and feed motor-safety */
	VictorSPX::Set(mode, demand0, demand1);
	_safetyHelper.Feed();
}
//----------------------- Invert routines -------------------//
/**
 * Common interface for inverting direction of a speed controller.
 *
 * @param isInverted The state of inversion, true is inverted.
 */
void WPI_VictorSPX::SetInverted(bool isInverted) {
	VictorSPX::SetInverted(isInverted);
}
/**
 * Common interface for returning the inversion state of a speed controller.
 *
 * @return isInverted The state of inversion, true is inverted.
 */
bool WPI_VictorSPX::GetInverted() const {
	return BaseMotorController::GetInverted();
}
//----------------------- turn-motor-off routines-------------------//
/**
 * Common interface for disabling a motor.
 */
void WPI_VictorSPX::Disable() {
	NeutralOutput();
}
/**
 * Common interface to stop the motor until Set is called again.
 */
void WPI_VictorSPX::StopMotor() {
	NeutralOutput();
}

//----------------------- Motor Safety-------------------//

/**
 * Set the safety expiration time.
 *
 * @param timeout The timeout (in seconds) for this motor object
 */
void WPI_VictorSPX::SetExpiration(double timeout) {
	_safetyHelper.SetExpiration(timeout);
}

/**
 * Return the safety expiration time.
 *
 * @return The expiration time value.
 */
double WPI_VictorSPX::GetExpiration() const {
	return _safetyHelper.GetExpiration();
}

/**
 * Check if the motor is currently alive or stopped due to a timeout.
 *
 * @return a bool value that is true if the motor has NOT timed out and should
 *         still be running.
 */
bool WPI_VictorSPX::IsAlive() const {
	return _safetyHelper.IsAlive();
}

/**
 * Check if motor safety is enabled.
 *
 * @return True if motor safety is enforced for this object
 */
bool WPI_VictorSPX::IsSafetyEnabled() const {
	return _safetyHelper.IsSafetyEnabled();
}

void WPI_VictorSPX::SetSafetyEnabled(bool enabled) {
	_safetyHelper.SetSafetyEnabled(enabled);
}

void WPI_VictorSPX::GetDescription(llvm::raw_ostream& desc) const {
	desc << _desc.c_str();
}

void WPI_VictorSPX::InitSendable(frc::SendableBuilder& builder) {
	builder.SetSmartDashboardType("Speed Controller");
	builder.SetSafeState([=]() {StopMotor();});
	builder.AddDoubleProperty("Value", [=]() {return Get();},
			[=](double value) {Set(value);});
}
