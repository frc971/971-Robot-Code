#pragma once

#include "ctre/phoenix/MotorControl/ControlMode.h"
#include "ctre/phoenix/MotorControl/ControlFrame.h"
#include "ctre/phoenix/MotorControl/NeutralMode.h"
#include "ctre/phoenix/MotorControl/FeedbackDevice.h"
#include "ctre/phoenix/MotorControl/StatusFrame.h"
#include "ctre/phoenix/MotorControl/LimitSwitchType.h"
#include "ctre/phoenix/MotorControl/Faults.h"
#include "ctre/phoenix/MotorControl/StickyFaults.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/Motion/TrajectoryPoint.h"
#include "ctre/phoenix/Motion/MotionProfileStatus.h"
#include "ctre/phoenix/ErrorCode.h"
#include "IFollower.h"

namespace ctre {
namespace phoenix {
namespace motorcontrol {

class IMotorControllerEnhanced: public virtual IMotorController {
public:
	virtual ~IMotorControllerEnhanced() {
	}

	//------ Set output routines. ----------//
	/* in parent */

	//------ Invert behavior ----------//
	/* in parent */

	//----- general output shaping ------------------//
	/* in parent */

	//------ Voltage Compensation ----------//
	/* in parent */

	//------ General Status ----------//
	/* in parent */

	//------ sensor selection ----------//
	/* expand the options */
	virtual ErrorCode ConfigSelectedFeedbackSensor(
			FeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) = 0;
	virtual ErrorCode ConfigSelectedFeedbackSensor(
			RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) = 0;

	//------- sensor status --------- //
	/* in parent */

	//------ status frame period changes ----------//
	virtual ErrorCode SetStatusFramePeriod(StatusFrame frame, int periodMs,
			int timeoutMs) = 0;
	virtual ErrorCode SetStatusFramePeriod(StatusFrameEnhanced frame,
			int periodMs, int timeoutMs) = 0;
	virtual int GetStatusFramePeriod(StatusFrame frame, int timeoutMs) = 0;
	virtual int GetStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs) = 0;

	//----- velocity signal conditionaing ------//
	virtual ErrorCode ConfigVelocityMeasurementPeriod(VelocityMeasPeriod period,
			int timeoutMs)= 0;
	virtual ErrorCode ConfigVelocityMeasurementWindow(int windowSize,
			int timeoutMs)= 0;

	//------ remote limit switch ----------//
	virtual ErrorCode ConfigForwardLimitSwitchSource(
			RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs) = 0;
	virtual ErrorCode ConfigReverseLimitSwitchSource(
			RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs) = 0;

	//------ local limit switch ----------//
	virtual ErrorCode ConfigForwardLimitSwitchSource(LimitSwitchSource type,
			LimitSwitchNormal normalOpenOrClose, int timeoutMs)= 0;
	virtual ErrorCode ConfigReverseLimitSwitchSource(LimitSwitchSource type,
			LimitSwitchNormal normalOpenOrClose, int timeoutMs)= 0;

	//------ soft limit ----------//
	/* in parent */

	//------ Current Lim ----------//
	virtual ErrorCode ConfigPeakCurrentLimit(int amps, int timeoutMs)= 0;
	virtual ErrorCode ConfigPeakCurrentDuration(int milliseconds,
			int timeoutMs)= 0;
	virtual ErrorCode ConfigContinuousCurrentLimit(int amps, int timeoutMs)= 0;
	virtual void EnableCurrentLimit(bool enable)= 0;

	//------ General Close loop ----------//
	/* in parent */

	//------ Motion Profile Settings used in Motion Magic and Motion Profile ----------//
	/* in parent */

	//------ Motion Profile Buffer ----------//
	/* in parent */

	//------ error ----------//
	/* in parent */

	//------ Faults ----------//
	/* in parent */

	//------ Firmware ----------//
	/* in parent */

	//------ Custom Persistent Params ----------//
	/* in parent */

	//------ Generic Param API, typically not used ----------//
	/* in parent */

	//------ Misc. ----------//
	/* in parent */

}; // class IMotorControllerEnhanced
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
