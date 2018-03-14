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
#pragma once

#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "SmartDashboard/SendableBase.h"
#include "SmartDashboard/SendableBuilder.h"
#include "SpeedController.h"
#include "MotorSafety.h"
#include "MotorSafetyHelper.h"

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {


class WPI_TalonSRX: public virtual TalonSRX,
		public virtual frc::SpeedController,
		public frc::SendableBase,
		public frc::MotorSafety {
public:
	WPI_TalonSRX(int deviceNumber);
	virtual ~WPI_TalonSRX();

	WPI_TalonSRX() = delete;
	WPI_TalonSRX(WPI_TalonSRX const&) = delete;
	WPI_TalonSRX& operator=(WPI_TalonSRX const&) = delete;

	//----------------------- set/get routines for WPILIB interfaces -------------------//
	/**
	 * Common interface for setting the speed of a simple speed controller.
	 *
	 * @param speed The speed to set.  Value should be between -1.0 and 1.0.
	 * 									Value is also saved for Get().
	 */
	virtual void Set(double speed);
	virtual void PIDWrite(double output);

	/**
	 * Common interface for getting the current set speed of a speed controller.
	 *
	 * @return The current set speed.  Value is between -1.0 and 1.0.
	 */
	virtual double Get() const;

	//----------------------- Intercept CTRE calls for motor safety -------------------//
	virtual void Set(ControlMode mode, double value);
	virtual void Set(ControlMode mode, double demand0, double demand1);
	//----------------------- Invert routines -------------------//
	/**
	 * Common interface for inverting direction of a speed controller.
	 *
	 * @param isInverted The state of inversion, true is inverted.
	 */
	virtual void SetInverted(bool isInverted);
	/**
	 * Common interface for returning the inversion state of a speed controller.
	 *
	 * @return isInverted The state of inversion, true is inverted.
	 */
	virtual bool GetInverted() const;
	//----------------------- turn-motor-off routines-------------------//
	/**
	 * Common interface for disabling a motor.
	 */
	virtual void Disable();
	/**
	 * Common interface to stop the motor until Set is called again.
	 */
	virtual void StopMotor();

	//----------------------- Motor Safety-------------------//

	/**
	 * Set the safety expiration time.
	 *
	 * @param timeout The timeout (in seconds) for this motor object
	 */
	void SetExpiration(double timeout);

	/**
	 * Return the safety expiration time.
	 *
	 * @return The expiration time value.
	 */
	double GetExpiration() const;

	/**
	 * Check if the motor is currently alive or stopped due to a timeout.
	 *
	 * @return a bool value that is true if the motor has NOT timed out and should
	 *         still be running.
	 */
	bool IsAlive() const;

	/**
	 * Check if motor safety is enabled.
	 *
	 * @return True if motor safety is enforced for this object
	 */
	bool IsSafetyEnabled() const;

	void SetSafetyEnabled(bool enabled);

	void GetDescription(llvm::raw_ostream& desc) const;

protected:
	virtual void InitSendable(frc::SendableBuilder& builder);
private:
	double _speed = 0;
	std::string _desc;
	frc::MotorSafetyHelper _safetyHelper;
};

} // namespace can
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
