#pragma once

#include "ctre/phoenix/ErrorCode.h"

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {
class BaseMotorController;
}
}
}
}

namespace ctre {
namespace phoenix {
namespace motorcontrol {

class SensorCollection {
public:

	/**
	 * Get the position of whatever is in the analog pin of the Talon, regardless of
	 *   whether it is actually being used for feedback.
	 *
	 * @return  the 24bit analog value.  The bottom ten bits is the ADC (0 - 1023)
	 *          on the analog pin of the Talon. The upper 14 bits tracks the overflows and underflows
	 *          (continuous sensor).
	 */

	int GetAnalogIn();

	/**
	 * Sets analog position.
	 *
	 * @param   newPosition The new position.
	 * @param   timeoutMs   
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
	 *
	 * @return  an ErrorCode.
	 */

	ErrorCode SetAnalogPosition(int newPosition, int timeoutMs);

	/**
	 * Get the position of whatever is in the analog pin of the Talon, regardless of whether
	 *   it is actually being used for feedback.
	 *
	 * @return  the ADC (0 - 1023) on analog pin of the Talon.
	 */

	int GetAnalogInRaw();

	/**
	 * Get the velocity of whatever is in the analog pin of the Talon, regardless of
	 *   whether it is actually being used for feedback.
	 *
	 * @return  the speed in units per 100ms where 1024 units is one rotation.
	 */

	int GetAnalogInVel();

	/**
	 * Get the quadrature position of the Talon, regardless of whether
	 *   it is actually being used for feedback.
	 *
	 * @return  the quadrature position.
	 */

	int GetQuadraturePosition();

	/**
	 * Change the quadrature reported position.  Typically this is used to "zero" the
	 *   sensor. This only works with Quadrature sensor.  To set the selected sensor position
	 *   regardless of what type it is, see SetSelectedSensorPosition in the motor controller class.
	 *
	 * @param   newPosition The position value to apply to the sensor.
	 * @param   timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
	 *
	 * @return  error code.
	 */

	ErrorCode SetQuadraturePosition(int newPosition, int timeoutMs);

	/**
	 * Get the quadrature velocity, regardless of whether
	 *   it is actually being used for feedback.
	 *
	 * @return  the quadrature velocity in units per 100ms.
	 */

	int GetQuadratureVelocity();

	/**
	 * Gets pulse width position, regardless of whether
	 *   it is actually being used for feedback.
	 *
	 * @return  the pulse width position.
	 */

	int GetPulseWidthPosition();

	/**
	 * Sets pulse width position.
	 *
	 * @param   newPosition The position value to apply to the sensor.
	 * @param   timeoutMs
 *            Timeout value in ms. If nonzero, function will wait for
 *            config success and report an error if it times out.
 *            If zero, no blocking or checking is performed.
	 *
	 * @return  an ErrErrorCode
	 */
	ErrorCode SetPulseWidthPosition(int newPosition, int timeoutMs);

	/**
	 * Gets pulse width velocity, regardless of whether
	 *   it is actually being used for feedback.
	 *
	 * @return  the pulse width velocity in units per 100ms (where 4096 units is 1 rotation).
	 */

	int GetPulseWidthVelocity();

	/**
	 * Gets pulse width rise to fall time.
	 *
	 * @return  the pulse width rise to fall time in microseconds.
	 */

	int GetPulseWidthRiseToFallUs();

	/**
	 * Gets pulse width rise to rise time.
	 *
	 * @return  the pulse width rise to rise time in microseconds.
	 */

	int GetPulseWidthRiseToRiseUs();

	/**
	 * Gets pin state quad a.
	 *
	 * @return  the pin state of quad a (1 if asserted, 0 if not asserted).
	 */

	int GetPinStateQuadA();

	/**
	 * Gets pin state quad b.
	 *
	 * @return  Digital level of QUADB pin (1 if asserted, 0 if not asserted).
	 */

	int GetPinStateQuadB();

	/**
	 * Gets pin state quad index.
	 *
	 * @return  Digital level of QUAD Index pin (1 if asserted, 0 if not asserted).
	 */

	int GetPinStateQuadIdx();

	/**
	 * Is forward limit switch closed.
	 *
	 * @return  '1' iff forward limit switch is closed, 0 iff switch is open. This function works
	 *          regardless if limit switch feature is enabled.
	 */

	int IsFwdLimitSwitchClosed();

	/**
	 * Is reverse limit switch closed.
	 *
	 * @return  '1' iff reverse limit switch is closed, 0 iff switch is open. This function works
	 *          regardless if limit switch feature is enabled.
	 */

	int IsRevLimitSwitchClosed();

private:
	SensorCollection(void * handle);
	friend class ctre::phoenix::motorcontrol::can::BaseMotorController;
	void* _handle;

};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
