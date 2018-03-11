#pragma once

#include "ctre/phoenix/Motion/SetValueMotionProfile.h"
#include "ctre/phoenix/Motion/TrajectoryPoint.h"

namespace ctre {
namespace phoenix {
namespace motion {

/**
 * Motion Profile Status
 * This is simply a data transer object.
 */
struct MotionProfileStatus {
	/**
	 * The available empty slots in the trajectory buffer.
	 *
	 * The robot API holds a "top buffer" of trajectory points, so your applicaion
	 * can dump several points at once.  The API will then stream them into the Talon's
	 * low-level buffer, allowing the Talon to act on them.
	 */
	int topBufferRem;
	/**
	 * The number of points in the top trajectory buffer.
	 */
	int topBufferCnt;
	/**
	 * The number of points in the low level Talon buffer.
	 */
	int btmBufferCnt;
	/**
	 * Set if isUnderrun ever gets set.
	 * Only is cleared by clearMotionProfileHasUnderrun() to ensure
	 * robot logic can react or instrument it.
	 * @see clearMotionProfileHasUnderrun()
	 */
	bool hasUnderrun;
	/**
	 * This is set if Talon needs to shift a point from its buffer into
	 * the active trajectory point however the buffer is empty. This gets cleared
	 * automatically when is resolved.
	 */
	bool isUnderrun;
	/**
	 * True if the active trajectory point has not empty, false otherwise.
	 * The members in activePoint are only valid if this signal is set.
	 */
	bool activePointValid;

	bool isLast;

	/** Selected slot for PID Loop 0 */
	int profileSlotSelect0;

	/** Selected slot for PID Loop 0 */
	int profileSlotSelect1;

	/**
	 * The current output mode of the motion profile executer (disabled, enabled, or hold).
	 * When changing the set() value in MP mode, it's important to check this signal to
	 * confirm the change takes effect before interacting with the top buffer.
	 */
	ctre::phoenix::motion::SetValueMotionProfile outputEnable;

	/** The applied duration of the active trajectory point */
	int timeDurMs;
};

} // namespace motion
} // namespace phoenix
} // namespace ctre

