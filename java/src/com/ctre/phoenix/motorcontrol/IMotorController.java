package com.ctre.phoenix.motorcontrol;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;

public interface IMotorController
		extends com.ctre.phoenix.signals.IOutputSignal, com.ctre.phoenix.signals.IInvertable, IFollower {
	// ------ Set output routines. ----------//
	public void set(ControlMode Mode, double demand);

	public void set(ControlMode Mode, double demand0, double demand1);

	public void set(ControlMode Mode, double demand0, DemandType demand1Type, double demand1);

	public void neutralOutput();

	public void setNeutralMode(NeutralMode neutralMode);

	// ------ Invert behavior ----------//
	public void setSensorPhase(boolean PhaseSensor);

	public void setInverted(boolean invert);

	public boolean getInverted();

	// ----- general output shaping ------------------//
	public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs);

	public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs);

	public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs);

	public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs);

	public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs);

	public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs);

	public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs);

	// ------ Voltage Compensation ----------//
	public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs);

	public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs);

	public void enableVoltageCompensation(boolean enable);

	// ------ General Status ----------//
	public double getBusVoltage() ;

	public double getMotorOutputPercent() ;

	public double getMotorOutputVoltage() ;

	public double getOutputCurrent() ;

	public double getTemperature() ;

	// ------ sensor selection ----------//
	public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs);

	public ErrorCode configSelectedFeedbackCoefficient(double coefficient, int pidIdx, int timeoutMs);

	public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal,
			int timeoutMs);

	public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs);

	// ------- sensor status --------- //
	public int getSelectedSensorPosition(int pidIdx);

	public int getSelectedSensorVelocity(int pidIdx);

	public ErrorCode setSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs);

	// ------ status frame period changes ----------//
	public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs);

	public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs);

	public int getStatusFramePeriod(StatusFrame frame, int timeoutMs);

	//----- velocity signal conditionaing ------//
	/* not supported */

	//------ remote limit switch ----------//
	public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs);

	public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs);

	public void overrideLimitSwitchesEnable(boolean enable);

	// ------ local limit switch ----------//
	/* not supported */

	// ------ soft limit ----------//
	public ErrorCode configForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs);

	public ErrorCode configReverseSoftLimitThreshold(int reverseSensorLimit, int timeoutMs);

	public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs);

	public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs);

	public void overrideSoftLimitsEnable(boolean enable);

	// ------ Current Lim ----------//
	/* not supported */

	// ------ General Close loop ----------//
	public ErrorCode config_kP(int slotIdx, double value, int timeoutMs);

	public ErrorCode config_kI(int slotIdx, double value, int timeoutMs);

	public ErrorCode config_kD(int slotIdx, double value, int timeoutMs);

	public ErrorCode config_kF(int slotIdx, double value, int timeoutMs);

	public ErrorCode config_IntegralZone(int slotIdx, int izone, int timeoutMs);

	public ErrorCode configAllowableClosedloopError(int slotIdx, int allowableCloseLoopError, int timeoutMs);

	public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs);

	public ErrorCode configClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs);

	public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs);

	public ErrorCode configAuxPIDPolarity(boolean invert, int timeoutMs);

	//------ Close loop State ----------//
	public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs);

	public int getClosedLoopError(int pidIdx);

	public double getIntegralAccumulator(int pidIdx) ;

	public double getErrorDerivative(int pidIdx) ;

	public void selectProfileSlot(int slotIdx, int pidIdx);

	public int getClosedLoopTarget(int pidIdx); // will be added to JNI

	public int getActiveTrajectoryPosition();

	public int getActiveTrajectoryVelocity();

	public double getActiveTrajectoryHeading();

	// ------ Motion Profile Settings used in Motion Magic and Motion Profile
	public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs);

	public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs);
	
	public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs);

	// ------ Motion Profile Buffer ----------//
	public ErrorCode clearMotionProfileTrajectories();
	public int getMotionProfileTopLevelBufferCount();
	public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt);
	public boolean isMotionProfileTopLevelBufferFull();
	public void processMotionProfileBuffer();
	public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill);
	public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs);
	public ErrorCode changeMotionControlFramePeriod(int periodMs);

	// ------ error ----------//
	public ErrorCode getLastError();

	// ------ Faults ----------//
	public ErrorCode getFaults(Faults toFill) ;

	public ErrorCode getStickyFaults(StickyFaults toFill) ;

	public ErrorCode clearStickyFaults(int timeoutMs);

	// ------ Firmware ----------//
	public int getFirmwareVersion();

	public boolean hasResetOccurred();

	// ------ Custom Persistent Params ----------//
	public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs);

	public int configGetCustomParam(int paramIndex, int timoutMs);

	//------ Generic Param API, typically not used ----------//
	public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal, int timeoutMs);
	public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal, int timeoutMs);

	public double configGetParameter(ParamEnum paramEnum, int ordinal, int timeoutMs) ;
	public double configGetParameter(int paramEnum, int ordinal, int timeoutMs) ;

	//------ Misc. ----------//
	public int getBaseID();
	public int getDeviceID();
	public ControlMode getControlMode();
	// ----- Follower ------//
	/* in parent interface */
}
