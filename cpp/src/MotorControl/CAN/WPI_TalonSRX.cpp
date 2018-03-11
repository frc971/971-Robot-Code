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
#include "ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h"
#include "ctre/phoenix/MotorControl/CAN/BaseMotorController.h"
#include "HAL/HAL.h"
#include <sstream>

using namespace ctre::phoenix::motorcontrol::can;


/** Constructor */
WPI_TalonSRX::WPI_TalonSRX(int deviceNumber) :
		BaseMotorController(deviceNumber | 0x02040000),
		TalonSRX(deviceNumber),
		_safetyHelper(this) {
	HAL_Report(HALUsageReporting::kResourceType_CTRE_future2, deviceNumber + 1);
	/* build string description */
	std::stringstream work;
	work << "Talon SRX " << deviceNumber;
	_desc = work.str();
	/* prep motor safety */
	_safetyHelper.SetExpiration(0.0);
	_safetyHelper.SetSafetyEnabled(false);
	SetName("Talon SRX ", deviceNumber);
}
WPI_TalonSRX::~WPI_TalonSRX() {
	/* MT */
}

//----------------------- set/get routines for WPILIB interfaces -------------------//
/**
 * Common interface for setting the speed of a simple speed controller.
 *
 * @param speed The speed to set.  Value should be between -1.0 and 1.0.
 * 									Value is also saved for Get().
 */
void WPI_TalonSRX::Set(double speed) {
	_speed = speed;
	TalonSRX::Set(ControlMode::PercentOutput, speed);
	_safetyHelper.Feed();
}
void WPI_TalonSRX::PIDWrite(double output) {
	Set(output);
}

/**
 * Common interface for getting the current set speed of a speed controller.
 *
 * @return The current set speed.  Value is between -1.0 and 1.0.
 */
double WPI_TalonSRX::Get() const {
	return _speed;
}

//----------------------- Intercept CTRE calls for motor safety -------------------//
void WPI_TalonSRX::Set(ControlMode mode, double value) {
	/* intercept the advanced Set and feed motor-safety */
	TalonSRX::Set(mode, value);
	_safetyHelper.Feed();
}
void WPI_TalonSRX::Set(ControlMode mode, double demand0, double demand1) {
	/* intercept the advanced Set and feed motor-safety */
	TalonSRX::Set(mode, demand0, demand1);
	_safetyHelper.Feed();
}
//----------------------- Invert routines -------------------//
/**
 * Common interface for inverting direction of a speed controller.
 *
 * @param isInverted The state of inversion, true is inverted.
 */
void WPI_TalonSRX::SetInverted(bool isInverted) {
	TalonSRX::SetInverted(isInverted);
}
/**
 * Common interface for returning the inversion state of a speed controller.
 *
 * @return isInverted The state of inversion, true is inverted.
 */
bool WPI_TalonSRX::GetInverted() const {
	return BaseMotorController::GetInverted();
}
//----------------------- turn-motor-off routines-------------------//
/**
 * Common interface for disabling a motor.
 */
void WPI_TalonSRX::Disable() {
	NeutralOutput();
}
/**
 * Common interface to stop the motor until Set is called again.
 */
void WPI_TalonSRX::StopMotor() {
	NeutralOutput();
}

//----------------------- Motor Safety-------------------//

/**
 * Set the safety expiration time.
 *
 * @param timeout The timeout (in seconds) for this motor object
 */
void WPI_TalonSRX::SetExpiration(double timeout) {
	_safetyHelper.SetExpiration(timeout);
}

/**
 * Return the safety expiration time.
 *
 * @return The expiration time value.
 */
double WPI_TalonSRX::GetExpiration() const {
	return _safetyHelper.GetExpiration();
}

/**
 * Check if the motor is currently alive or stopped due to a timeout.
 *
 * @return a bool value that is true if the motor has NOT timed out and should
 *         still be running.
 */
bool WPI_TalonSRX::IsAlive() const {
	return _safetyHelper.IsAlive();
}

/**
 * Check if motor safety is enabled.
 *
 * @return True if motor safety is enforced for this object
 */
bool WPI_TalonSRX::IsSafetyEnabled() const {
	return _safetyHelper.IsSafetyEnabled();
}

void WPI_TalonSRX::SetSafetyEnabled(bool enabled) {
	_safetyHelper.SetSafetyEnabled(enabled);
}

void WPI_TalonSRX::GetDescription(llvm::raw_ostream& desc) const {
	desc << _desc.c_str();
}

void WPI_TalonSRX::InitSendable(frc::SendableBuilder& builder) {
	builder.SetSmartDashboardType("Speed Controller");
	builder.SetSafeState([=]() {StopMotor();});
	builder.AddDoubleProperty("Value", [=]() {return Get();},
			[=](double value) {Set(value);});
}
