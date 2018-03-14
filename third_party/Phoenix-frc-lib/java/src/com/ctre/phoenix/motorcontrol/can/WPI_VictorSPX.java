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
package com.ctre.phoenix.motorcontrol.can;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.hal.HAL;

public class WPI_VictorSPX extends VictorSPX implements SpeedController, Sendable, MotorSafety {

	private String _description;
	private double _speed;
	private MotorSafetyHelper _safetyHelper;

	/** Constructor */
	public WPI_VictorSPX(int deviceNumber) {
		super(deviceNumber);
		HAL.report(67, deviceNumber + 1);
		_description = "Victor SPX " + deviceNumber;
		/* prep motor safety */
		_safetyHelper = new MotorSafetyHelper(this);
		_safetyHelper.setExpiration(0.0);
		_safetyHelper.setSafetyEnabled(false);

		LiveWindow.add(this);
		setName("Victor SPX ", deviceNumber);
	}

	// ------ set/get routines for WPILIB interfaces ------//
	@Override
	public void set(double speed) {
		_speed = speed;
		set(ControlMode.PercentOutput, _speed);
		_safetyHelper.feed();
	}

	@Override
	public void pidWrite(double output) {
		set(output);
	}

	/**
	 * Common interface for getting the current set speed of a speed controller.
	 *
	 * @return The current set speed. Value is between -1.0 and 1.0.
	 */
	@Override
	public double get() {
		return _speed;
	}

	// ---------Intercept CTRE calls for motor safety ---------//
	public void set(ControlMode mode, double value) {
		/* intercept the advanced Set and feed motor-safety */
		super.set(mode, value);
		_safetyHelper.feed();
	}

	public void set(ControlMode mode, double demand0, double demand1) {
		/* intercept the advanced Set and feed motor-safety */
		super.set(mode, demand0, demand1);
		_safetyHelper.feed();
	}

	// ----------------------- Invert routines -------------------//
	@Override
	public void setInverted(boolean isInverted) {
		super.setInverted(isInverted);
	}

	@Override
	public boolean getInverted() {
		return super.getInverted();
	}

	// ----------------------- turn-motor-off routines-------------------//
	@Override
	public void disable() {
		neutralOutput();
	}

	/**
	 * Common interface to stop the motor until Set is called again.
	 */
	@Override
	public void stopMotor() {
		neutralOutput();
	}
	// -------- Motor Safety--------//

	/**
	 * Set the safety expiration time.
	 *
	 * @param timeout
	 *            The timeout (in seconds) for this motor object
	 */
	@Override
	public void setExpiration(double timeout) {
		_safetyHelper.setExpiration(timeout);
	}

	/**
	 * Return the safety expiration time.
	 *
	 * @return The expiration time value.
	 */
	@Override
	public double getExpiration() {
		return _safetyHelper.getExpiration();
	}

	/**
	 * Check if the motor is currently alive or stopped due to a timeout.
	 *
	 * @return a bool value that is true if the motor has NOT timed out and
	 *         should still be running.
	 */
	@Override
	public boolean isAlive() {
		return _safetyHelper.isAlive();
	}

	/**
	 * Check if motor safety is enabled.
	 *
	 * @return True if motor safety is enforced for this object
	 */
	@Override
	public boolean isSafetyEnabled() {
		return _safetyHelper.isSafetyEnabled();
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
		_safetyHelper.setSafetyEnabled(enabled);
	}

	// ---- essentially a copy of SendableBase -------//
	private String m_name = "";
	private String m_subsystem = "Ungrouped";

	/**
	 * Free the resources used by this object.
	 */
	public void free() {
		LiveWindow.remove(this);
	}

	@Override
	public final synchronized String getName() {
		return m_name;
	}

	@Override
	public final synchronized void setName(String name) {
		m_name = name;
	}

	/**
	 * Sets the name of the sensor with a channel number.
	 *
	 * @param moduleType
	 *            A string that defines the module name in the label for the
	 *            value
	 * @param channel
	 *            The channel number the device is plugged into
	 */
	protected final void setName(String moduleType, int channel) {
		setName(moduleType + "[" + channel + "]");
	}

	/**
	 * Sets the name of the sensor with a module and channel number.
	 *
	 * @param moduleType
	 *            A string that defines the module name in the label for the
	 *            value
	 * @param moduleNumber
	 *            The number of the particular module type
	 * @param channel
	 *            The channel number the device is plugged into (usually PWM)
	 */
	protected final void setName(String moduleType, int moduleNumber, int channel) {
		setName(moduleType + "[" + moduleNumber + "," + channel + "]");
	}

	@Override
	public final synchronized String getSubsystem() {
		return m_subsystem;
	}

	@Override
	public final synchronized void setSubsystem(String subsystem) {
		m_subsystem = subsystem;
	}

	/**
	 * Add a child component.
	 *
	 * @param child
	 *            child component
	 */
	protected final void addChild(Object child) {
		LiveWindow.addChild(this, child);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Speed Controller");
		builder.setSafeState(this::stopMotor);
		builder.addDoubleProperty("Value", this::get, this::set);
	}

	@Override
	public String getDescription() {
		return _description;
	}
}
