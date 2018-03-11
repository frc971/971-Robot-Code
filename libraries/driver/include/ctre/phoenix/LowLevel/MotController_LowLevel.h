#pragma once

#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/LowLevel/Device_LowLevel.h"
#include "ctre/phoenix/MotorControl/FeedbackDevice.h"
#include "ctre/phoenix/MotorControl/ControlFrame.h"
#include "ctre/phoenix/MotorControl/SensorTerm.h"
#include "ctre/phoenix/MotorControl/RemoteSensorSource.h"
#include "ctre/phoenix/MotorControl/Faults.h"
#include "ctre/phoenix/MotorControl/StickyFaults.h"
#include "ctre/phoenix/MotorControl/NeutralMode.h"
#include "ctre/phoenix/MotorControl/ControlMode.h"
#include "ctre/phoenix/MotorControl/LimitSwitchType.h"
#include "ctre/phoenix/MotorControl/StatusFrame.h"
#include "ctre/phoenix/MotorControl/VelocityMeasPeriod.h"
#include <string>
#include <stdint.h>

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace lowlevel {

class MotController_LowLevel: public Device_LowLevel {

protected:
	const uint32_t STATUS_01 = 0x041400;
	const uint32_t STATUS_02 = 0x041440;
	const uint32_t STATUS_03 = 0x041480;
	const uint32_t STATUS_04 = 0x0414C0;
	const uint32_t STATUS_05 = 0x041500;
	const uint32_t STATUS_06 = 0x041540;
	const uint32_t STATUS_07 = 0x041580;
	const uint32_t STATUS_08 = 0x0415C0;
	const uint32_t STATUS_09 = 0x041600;
	const uint32_t STATUS_10 = 0x041640;
	const uint32_t STATUS_11 = 0x041680;
	const uint32_t STATUS_12 = 0x0416C0;
	const uint32_t STATUS_13 = 0x041700;
	const uint32_t STATUS_14 = 0x041740;
	const uint32_t STATUS_15 = 0x041780;

	const uint32_t CONTROL_1 = 0x040000;
	//const uint32_t CONTROL_2 = 0x040040;
	const uint32_t CONTROL_3 = 0x040080;
	const uint32_t CONTROL_5 = 0x040100;
	const uint32_t CONTROL_6 = 0x040140;

	const double FLOAT_TO_FXP_10_22 = (double) 0x400000;
	const double FXP_TO_FLOAT_10_22 = 0.0000002384185791015625f;

	const double FLOAT_TO_FXP_0_8 = (double) 0x100;
	const double FXP_TO_FLOAT_0_8 = 0.00390625f;

	/* Motion Profile Set Output */
	// Motor output is neutral, Motion Profile Executer is not running.
	const int kMotionProf_Disabled = 0;
	// Motor output is updated from Motion Profile Executer, MPE will
	// process the buffered points.
	const int kMotionProf_Enable = 1;
	// Motor output is updated from Motion Profile Executer, MPE will
	// stay processing current trajectory point.
	const int kMotionProf_Hold = 2;

	const int kDefaultControl6PeriodMs = 10;

	void EnableFirmStatusFrame(bool enable);
	ErrorCode SetLastError(ErrorCode errorCode);
	ErrorCode SetLastError(int errorCode);

	ErrorCode ConfigSingleLimitSwitchSource(
			LimitSwitchSource limitSwitchSource, LimitSwitchNormal normalOpenOrClose,
			int deviceIDIfApplicable, int timeoutMs, bool isForward);
public:
	MotController_LowLevel(int baseArbId);

	ErrorCode SetDemand(ctre::phoenix::motorcontrol::ControlMode mode, int demand0, int demand1);
	ErrorCode Set(ctre::phoenix::motorcontrol::ControlMode mode, double demand0, double demand1, int demand1Type);
	void SelectDemandType(bool enable);
	void SetMPEOutput(int MpeOutput);
	void EnableHeadingHold(bool enable);
	void SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode neutralMode);
	void SetSensorPhase(bool PhaseSensor);
	void SetInverted(bool invert);

	ErrorCode ConfigOpenLoopRamp(double secondsFromNeutralToFull, int timeoutMs);
	ErrorCode ConfigClosedLoopRamp(double secondsFromNeutralToFull,
			int timeoutMs);
	ErrorCode ConfigPeakOutputForward(double percentOut, int timeoutMs);
	ErrorCode ConfigPeakOutputReverse(double percentOut, int timeoutMs);
	ErrorCode ConfigNominalOutputForward(double percentOut, int timeoutMs);
	ErrorCode ConfigNominalOutputReverse(double percentOut, int timeoutMs);
	ErrorCode ConfigNeutralDeadband(double percentDeadband,
			int timeoutMs);
	ErrorCode ConfigVoltageCompSaturation(double voltage, int timeoutMs);
	ErrorCode ConfigVoltageMeasurementFilter(int filterWindowSamples,
			int timeoutMs);
	void EnableVoltageCompensation(bool enable);
	ErrorCode GetBusVoltage(double & param);
	ErrorCode GetMotorOutputPercent(double & param);
	ErrorCode GetOutputCurrent(double & param);
	ErrorCode GetTemperature(double & param);
	ErrorCode ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice feedbackDevice,
			int pidIdx, int timeoutMs);
	ErrorCode ConfigSelectedFeedbackCoefficient(double coefficient, int pidIdx, int timeoutMs);
	ErrorCode ConfigRemoteFeedbackFilter(int deviceID,
			RemoteSensorSource remoteSensorSource, int remoteOrdinal, int timeoutMs);
	ErrorCode ConfigSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs);
	ErrorCode GetSelectedSensorPosition(int & param, int pidIdx);
	ErrorCode GetSelectedSensorVelocity(int & param, int pidIdx);
	ErrorCode SetSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs);
	ErrorCode SetControlFramePeriod(
			ctre::phoenix::motorcontrol::ControlFrame frame, int periodMs);
	ErrorCode SetStatusFramePeriod(
			ctre::phoenix::motorcontrol::StatusFrame frame, int periodMs,
			int timeoutMs);
	ErrorCode SetStatusFramePeriod(
			ctre::phoenix::motorcontrol::StatusFrameEnhanced frame,
			int periodMs, int timeoutMs);
	ErrorCode GetStatusFramePeriod(
			ctre::phoenix::motorcontrol::StatusFrame frame, int & periodMs,
			int timeoutMs);
	ErrorCode GetStatusFramePeriod(
			ctre::phoenix::motorcontrol::StatusFrameEnhanced frame,
			int & periodMs, int timeoutMs);
	ErrorCode ConfigVelocityMeasurementPeriod(
			ctre::phoenix::motorcontrol::VelocityMeasPeriod period,
			int timeoutMs);
	ErrorCode ConfigVelocityMeasurementWindow(int windowSize, int timeoutMs);
	ErrorCode ConfigForwardLimitSwitchSource(
			ctre::phoenix::motorcontrol::LimitSwitchSource type,
			ctre::phoenix::motorcontrol::LimitSwitchNormal normalOpenOrClose,
			int deviceIDIfApplicable, int timeoutMs);
	ErrorCode ConfigReverseLimitSwitchSource(
			ctre::phoenix::motorcontrol::LimitSwitchSource type,
			ctre::phoenix::motorcontrol::LimitSwitchNormal normalOpenOrClose,
			int deviceIDIfApplicable, int timeoutMs);
	void OverrideLimitSwitchesEnable(bool enable);
	ErrorCode ConfigForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs);
	ErrorCode ConfigReverseSoftLimitThreshold(int reverseSensorLimit, int timeoutMs);
	ErrorCode ConfigForwardSoftLimitEnable(bool enable, int timeoutMs);
	ErrorCode ConfigReverseSoftLimitEnable(bool enable, int timeoutMs);
	void OverrideSoftLimitsEnable(bool enable);
	ErrorCode ConfigPeakCurrentLimit(int amps, int timeoutMs);
	ErrorCode ConfigPeakCurrentDuration(int milliseconds, int timeoutMs);
	ErrorCode ConfigContinuousCurrentLimit(int amps, int timeoutMs);
	void EnableCurrentLimit(bool enable);
	ErrorCode Config_kP(int slotIdx, double value, int timeoutMs);
	ErrorCode Config_kI(int slotIdx, double value, int timeoutMs);
	ErrorCode Config_kD(int slotIdx, double value, int timeoutMs);
	ErrorCode Config_kF(int slotIdx, double value, int timeoutMs);
	ErrorCode Config_IntegralZone(int slotIdx, int izone, int timeoutMs);
	ErrorCode ConfigAllowableClosedloopError(int slotIdx,
			int allowableCloseLoopError, int timeoutMs);
	ErrorCode ConfigMaxIntegralAccumulator(int slotIdx, double iaccum,
			int timeoutMs);
	ErrorCode ConfigClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs);
	ErrorCode ConfigClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs);
	ErrorCode SetIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs);
	ErrorCode GetClosedLoopError(int & error, int pidIdx);
	ErrorCode GetIntegralAccumulator(double & iaccum, int pidIdx);
	ErrorCode GetErrorDerivative(double & derivError, int pidIdx);
	ErrorCode SelectProfileSlot(int slotIdx, int pidIdx);

	ErrorCode GetFaults(ctre::phoenix::motorcontrol::Faults & toFill);
	ErrorCode GetStickyFaults(ctre::phoenix::motorcontrol::StickyFaults& toFill);
	ErrorCode ClearStickyFaults(int timeoutMs);

	ErrorCode GetAnalogInWithOv(int & param);
	ErrorCode GetAnalogInVel(int & param);
	ErrorCode GetAnalogInAll(int & withOv, int & vRaw, int & vel);
	ErrorCode GetQuadraturePosition(int & param);
	ErrorCode GetQuadratureVelocity(int & param);
	ErrorCode GetQuadratureSensor(int & pos, int & vel);
	ErrorCode GetPulseWidthPosition(int & param);
	ErrorCode GetPulseWidthVelocity(int & param);
	ErrorCode GetPulseWidthRiseToFallUs(int & param);
	ErrorCode GetPulseWidthRiseToRiseUs(int & param);
	ErrorCode GetPulseWidthAll(int & pos, int & vel, int & riseToRiseUs, int & riseToFallUs);
	ErrorCode GetPinStateQuadA(int & param);
	ErrorCode GetPinStateQuadB(int & param);
	ErrorCode GetPinStateQuadIdx(int & param);
	ErrorCode GetQuadPinStates(int & quadA, int & quadB, int & quadIdx);
	ErrorCode IsFwdLimitSwitchClosed(int & param);
	ErrorCode IsRevLimitSwitchClosed(int & param);
	ErrorCode GetLimitSwitchState(int & isFwdClosed, int & isRevClosed);
	ErrorCode GetLastError();

	ErrorCode ConfigMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs);
	ErrorCode ConfigMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs);

	ErrorCode GetClosedLoopTarget(int & value, int pidIdx);
	ErrorCode GetActiveTrajectoryPosition(int & sensorUnits);
	ErrorCode GetActiveTrajectoryVelocity(int & sensorUnitsPer100ms);
	ErrorCode GetActiveTrajectoryHeading(double & turnUnits);
	ErrorCode GetActiveTrajectoryAll(int & vel, int & pos, double & heading);

	ErrorCode SetAnalogPosition(int newPosition, int timeoutMs);
	ErrorCode SetQuadraturePosition(int newPosition, int timeoutMs);
	ErrorCode SetPulseWidthPosition(int newPosition, int timeoutMs);

	const static int kMinFirmwareVersionMajor = 3;
	const static int kMinFirmwareVersionMinor = 0;

private:

	uint64_t _cache = 0;
	int32_t _len = 0;
	ErrorCode _lastError = (ErrorCode)0;
	int32_t _setPoint = 0;
	ControlMode _appliedMode = ControlMode::Disabled;
	void CheckFirmVers(int minMajor = kMinFirmwareVersionMajor, int minMinor = kMinFirmwareVersionMinor, 
			ErrorCode code = ErrorCode::FirmwareTooOld);
	int _usingAdvancedFeatures = 0;

};

}
}
}
}
