package com.ctre.phoenix.motorcontrol;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;

public class SensorCollection {

	private long _handle;

	public SensorCollection(BaseMotorController motorController) {
		_handle = motorController.getHandle();

	}

	/**
	 * Get the position of whatever is in the analog pin of the Talon, regardless of
	 *   whether it is actually being used for feedback.
	 *
	 * @return  the 24bit analog value.  The bottom ten bits is the ADC (0 - 1023)
	 *          on the analog pin of the Talon. The upper 14 bits tracks the overflows and underflows
	 *          (continuous sensor).
	 */

	public int getAnalogIn() {
		return MotControllerJNI.GetAnalogIn(_handle);
	}

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

	public ErrorCode setAnalogPosition(int newPosition, int timeoutMs) {
		int retval = MotControllerJNI.SetAnalogPosition(_handle, newPosition, timeoutMs);
		return ErrorCode.valueOf(retval);
	}

	/**
	 * Get the position of whatever is in the analog pin of the Talon, regardless of whether
	 *   it is actually being used for feedback.
	 *
	 * @return  the ADC (0 - 1023) on analog pin of the Talon.
	 */

	public int getAnalogInRaw() {
		return MotControllerJNI.GetAnalogInRaw(_handle);
	}

	/**
	 * Get the velocity of whatever is in the analog pin of the Talon, regardless of
	 *   whether it is actually being used for feedback.
	 *
	 * @return  the speed in units per 100ms where 1024 units is one rotation.
	 */

	public int getAnalogInVel() {
		return MotControllerJNI.GetAnalogInVel(_handle);
	}

	/**
	 * Get the quadrature position of the Talon, regardless of whether
	 *   it is actually being used for feedback.
	 *
	 * @return  the quadrature position.
	 */

	public int getQuadraturePosition() {
		return MotControllerJNI.GetQuadraturePosition(_handle);
	}

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

	public ErrorCode setQuadraturePosition(int newPosition, int timeoutMs) {
		int retval = MotControllerJNI.SetQuadraturePosition(_handle, newPosition, timeoutMs);
		return ErrorCode.valueOf(retval);
	}

	/**
	 * Get the quadrature velocity, regardless of whether
	 *   it is actually being used for feedback.
	 *
	 * @return  the quadrature velocity in units per 100ms.
	 */

	public int getQuadratureVelocity() {
		return MotControllerJNI.GetQuadratureVelocity(_handle);
	}

	/**
	 * Gets pulse width position, regardless of whether
	 *   it is actually being used for feedback.
	 *
	 * @return  the pulse width position.
	 */

	public int getPulseWidthPosition() {
		return MotControllerJNI.GetPulseWidthPosition(_handle);
	}

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
	public ErrorCode setPulseWidthPosition(int newPosition, int timeoutMs) {
		int retval = MotControllerJNI.SetPulseWidthPosition(_handle, newPosition, timeoutMs);
		return ErrorCode.valueOf(retval);
	}

	/**
	 * Gets pulse width velocity, regardless of whether
	 *   it is actually being used for feedback.
	 *
	 * @return  the pulse width velocity in units per 100ms (where 4096 units is 1 rotation).
	 */

	public int getPulseWidthVelocity() {
		return MotControllerJNI.GetPulseWidthVelocity(_handle);
	}

	/**
	 * Gets pulse width rise to fall time.
	 *
	 * @return  the pulse width rise to fall time in microseconds.
	 */

	public int getPulseWidthRiseToFallUs() {
		return MotControllerJNI.GetPulseWidthRiseToFallUs(_handle);
	}

	/**
	 * Gets pulse width rise to rise time.
	 *
	 * @return  the pulse width rise to rise time in microseconds.
	 */

	public int getPulseWidthRiseToRiseUs() {
		return MotControllerJNI.GetPulseWidthRiseToRiseUs(_handle);
	}

	/**
	 * Gets pin state quad a.
	 *
	 * @return  the pin state of quad a (1 if asserted, 0 if not asserted).
	 */

	public boolean getPinStateQuadA() {
		return MotControllerJNI.GetPinStateQuadA(_handle) != 0;
	}

	/**
	 * Gets pin state quad b.
	 *
	 * @return  Digital level of QUADB pin (1 if asserted, 0 if not asserted).
	 */

	public boolean getPinStateQuadB() {
		return MotControllerJNI.GetPinStateQuadB(_handle) != 0;
	}

	/**
	 * Gets pin state quad index.
	 *
	 * @return  Digital level of QUAD Index pin (1 if asserted, 0 if not asserted).
	 */

	public boolean getPinStateQuadIdx() {
		return MotControllerJNI.GetPinStateQuadIdx(_handle) != 0;
	}

	/**
	 * Is forward limit switch closed.
	 *
	 * @return  '1' iff forward limit switch is closed, 0 iff switch is open. This function works
	 *          regardless if limit switch feature is enabled.
	 */

	public boolean isFwdLimitSwitchClosed() {
		return MotControllerJNI.IsFwdLimitSwitchClosed(_handle) != 0;
	}

	/**
	 * Is reverse limit switch closed.
	 *
	 * @return  '1' iff reverse limit switch is closed, 0 iff switch is open. This function works
	 *          regardless if limit switch feature is enabled.
	 */

	public boolean isRevLimitSwitchClosed() {
		return MotControllerJNI.IsRevLimitSwitchClosed(_handle) != 0;
	}
}
