#pragma once

#include "ctre/phoenix/MotorControl/ControlMode.h"
#include "ctre/phoenix/MotorControl/ControlFrame.h"
#include "ctre/phoenix/MotorControl/DemandType.h"
#include "ctre/phoenix/MotorControl/NeutralMode.h"
#include "ctre/phoenix/MotorControl/FeedbackDevice.h"
#include "ctre/phoenix/MotorControl/RemoteSensorSource.h"
#include "ctre/phoenix/MotorControl/SensorTerm.h"
#include "ctre/phoenix/MotorControl/StatusFrame.h"
#include "ctre/phoenix/MotorControl/LimitSwitchType.h"
#include "ctre/phoenix/MotorControl/Faults.h"
#include "ctre/phoenix/MotorControl/StickyFaults.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/Motion/TrajectoryPoint.h"
#include "ctre/phoenix/Motion/MotionProfileStatus.h"
#include "ctre/phoenix/ErrorCode.h"
#include "IFollower.h"
/* WPILIB */
#include "SpeedController.h"

namespace ctre {
namespace phoenix {
namespace motorcontrol {

class IMotorController: public virtual IFollower {
public:
	virtual ~IMotorController() {
	}
	//------ Set output routines. ----------//
	virtual void Set(ControlMode Mode, double demand) = 0;
	virtual void Set(ControlMode Mode, double demand0, double demand1) = 0;
	virtual void Set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) = 0;
	virtual void NeutralOutput() = 0;
	virtual void SetNeutralMode(NeutralMode neutralMode) = 0;

	//------ Invert behavior ----------//
	virtual void SetSensorPhase(bool PhaseSensor) = 0;
	virtual void SetInverted(bool invert) = 0;
	virtual bool GetInverted() const = 0;

	//----- general output shaping ------------------//
	virtual ErrorCode ConfigOpenloopRamp(double secondsFromNeutralToFull,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigClosedloopRamp(double secondsFromNeutralToFull,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigPeakOutputForward(double percentOut,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigPeakOutputReverse(double percentOut,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigNominalOutputForward(double percentOut,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigNominalOutputReverse(double percentOut,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigNeutralDeadband(double percentDeadband,
			int timeoutMs) = 0;

	//------ Voltage Compensation ----------//
	virtual ErrorCode ConfigVoltageCompSaturation(double voltage,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigVoltageMeasurementFilter(int filterWindowSamples,
			int timeoutMs) = 0;
	virtual void EnableVoltageCompensation(bool enable) = 0;

	//------ General Status ----------//
	virtual double GetBusVoltage() = 0;
	virtual double GetMotorOutputPercent() = 0;
	virtual double GetMotorOutputVoltage() = 0;
	virtual double GetOutputCurrent() = 0;
	virtual double GetTemperature() = 0;

	//------ sensor selection ----------//
	virtual ErrorCode ConfigSelectedFeedbackSensor(
			RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) = 0;
	virtual ErrorCode ConfigSelectedFeedbackCoefficient(
			double coefficient, int pidIdx, int timeoutMs) = 0;
	virtual ErrorCode ConfigRemoteFeedbackFilter(int deviceID,
			RemoteSensorSource remoteSensorSource, int remoteOrdinal,
			int timeoutMs)= 0;
	virtual ErrorCode ConfigSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs)= 0;

	//------- sensor status --------- //
	virtual int GetSelectedSensorPosition(int pidIdx) = 0;
	virtual int GetSelectedSensorVelocity(int pidIdx) = 0;
	virtual ErrorCode SetSelectedSensorPosition(int sensorPos, int pidIdx,
			int timeoutMs) = 0;

	//------ status frame period changes ----------//
	virtual ErrorCode SetControlFramePeriod(ControlFrame frame,
			int periodMs) = 0;
	virtual ErrorCode SetStatusFramePeriod(StatusFrame frame, int periodMs,
			int timeoutMs) = 0;
	virtual int GetStatusFramePeriod(StatusFrame frame, int timeoutMs) = 0;

	//----- velocity signal conditionaing ------//
	/* not supported */

	//------ remote limit switch ----------//
	virtual ErrorCode ConfigForwardLimitSwitchSource(
			RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs) = 0;
	virtual ErrorCode ConfigReverseLimitSwitchSource(
			RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs) = 0;
	virtual void OverrideLimitSwitchesEnable(bool enable) = 0;

	//------ local limit switch ----------//
	/* not supported */

	//------ soft limit ----------//
	virtual ErrorCode ConfigForwardSoftLimitThreshold(int forwardSensorLimit,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigReverseSoftLimitThreshold(int reverseSensorLimit,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigForwardSoftLimitEnable(bool enable,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigReverseSoftLimitEnable(bool enable,
			int timeoutMs) = 0;
	virtual void OverrideSoftLimitsEnable(bool enable) = 0;

	//------ Current Lim ----------//
	/* not supported */

	//------ Config Close loop ----------//
	virtual ErrorCode Config_kP(int slotIdx, double value, int timeoutMs) = 0;
	virtual ErrorCode Config_kI(int slotIdx, double value, int timeoutMs) = 0;
	virtual ErrorCode Config_kD(int slotIdx, double value, int timeoutMs) = 0;
	virtual ErrorCode Config_kF(int slotIdx, double value, int timeoutMs) = 0;
	virtual ErrorCode Config_IntegralZone(int slotIdx, int izone,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigAllowableClosedloopError(int slotIdx,
			int allowableCloseLoopError, int timeoutMs) = 0;
	virtual ErrorCode ConfigMaxIntegralAccumulator(int slotIdx, double iaccum,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs) = 0;
	virtual ErrorCode ConfigClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs) = 0;
  virtual ErrorCode ConfigAuxPIDPolarity(bool invert, int timeoutMs) = 0;

	//------ Close loop State ----------//
	virtual ErrorCode SetIntegralAccumulator(double iaccum, int pidIdx,
			int timeoutMs) = 0;
	virtual int GetClosedLoopError(int pidIdx) = 0;
	virtual double GetIntegralAccumulator(int pidIdx) = 0;
	virtual double GetErrorDerivative(int pidIdx) = 0;

	virtual ErrorCode SelectProfileSlot(int slotIdx, int pidIdx) = 0;

	virtual int GetClosedLoopTarget(int pidIdx) = 0;
	virtual int GetActiveTrajectoryPosition() = 0;
	virtual int GetActiveTrajectoryVelocity() = 0;
	virtual double GetActiveTrajectoryHeading() = 0;

	//------ Motion Profile Settings used in Motion Magic  ----------//
	virtual ErrorCode ConfigMotionCruiseVelocity(int sensorUnitsPer100ms,
			int timeoutMs) = 0;
	virtual ErrorCode ConfigMotionAcceleration(int sensorUnitsPer100msPerSec,
			int timeoutMs) = 0;

	//------ Motion Profile Buffer ----------//
	virtual ErrorCode ClearMotionProfileTrajectories()= 0;
	virtual int GetMotionProfileTopLevelBufferCount()= 0;
	virtual ErrorCode PushMotionProfileTrajectory(
			const ctre::phoenix::motion::TrajectoryPoint & trajPt)= 0;
	virtual bool IsMotionProfileTopLevelBufferFull()= 0;
	virtual void ProcessMotionProfileBuffer()= 0;
	virtual ErrorCode GetMotionProfileStatus(
			ctre::phoenix::motion::MotionProfileStatus & statusToFill)= 0;
	virtual ErrorCode ClearMotionProfileHasUnderrun(int timeoutMs)= 0;
	virtual ErrorCode ChangeMotionControlFramePeriod(int periodMs)= 0;
	virtual ErrorCode ConfigMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs)=0;
	//------ error ----------//
	virtual ErrorCode GetLastError() = 0;

	//------ Faults ----------//
	virtual ErrorCode GetFaults(Faults & toFill) = 0;
	virtual ErrorCode GetStickyFaults(StickyFaults & toFill) = 0;
	virtual ErrorCode ClearStickyFaults(int timeoutMs) = 0;

	//------ Firmware ----------//
	virtual int GetFirmwareVersion() = 0;
	virtual bool HasResetOccurred() = 0;

	//------ Custom Persistent Params ----------//
	virtual ErrorCode ConfigSetCustomParam(int newValue, int paramIndex,
			int timeoutMs) = 0;
	virtual int ConfigGetCustomParam(int paramIndex, int timeoutMs) = 0;

	//------ Generic Param API, typically not used ----------//
	virtual ErrorCode ConfigSetParameter(ParamEnum param, double value,
			uint8_t subValue, int ordinal, int timeoutMs) = 0;
	virtual double ConfigGetParameter(ParamEnum paramEnum, int ordinal,
			int timeoutMs) = 0;

	//------ Misc. ----------//
	virtual int GetBaseID() = 0;
	virtual int GetDeviceID() = 0;
	virtual ControlMode GetControlMode() = 0;

	// ----- Follower ------//
	/* in parent interface */
};

}
} // namespace phoenix
}
