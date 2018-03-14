package com.ctre.phoenix.motion;

/**
 * Motion Profile Status This is simply a data transer object.
 */
public class MotionProfileStatus {
	/**
	 * The available empty slots in the trajectory buffer.
	 *
	 * The robot API holds a "top buffer" of trajectory points, so your
	 * applicaion can dump several points at once. The API will then stream them
	 * into the Talon's low-level buffer, allowing the Talon to act on them.
	 */
	public int topBufferRem;
	/**
	 * The number of points in the top trajectory buffer.
	 */
	public int topBufferCnt;
	/**
	 * The number of points in the low level Talon buffer.
	 */
	public int btmBufferCnt;
	/**
	 * Set if isUnderrun ever gets set. Only is cleared by
	 * clearMotionProfileHasUnderrun() to ensure robot logic can react or
	 * instrument it.
	 *
	 * @see com.ctre.phoenix.motorcontrol.can.BaseMotorController#clearMotionProfileHasUnderrun(int)
	 */
	public boolean hasUnderrun;
	/**
	 * This is set if Talon needs to shift a point from its buffer into the
	 * active trajectory point however the buffer is empty. This gets cleared
	 * automatically when is resolved.
	 */
	public boolean isUnderrun;
	/**
	 * True if the active trajectory point has not empty, false otherwise. The
	 * members in activePoint are only valid if this signal is set.
	 */
	public boolean activePointValid;

	public boolean isLast;

	public int profileSlotSelect;
	/**
	 * The current output mode of the motion profile executer (disabled,
	 * enabled, or hold). When changing the set() value in MP mode, it's
	 * important to check this signal to confirm the change takes effect before
	 * interacting with the top buffer.
	 */
	public SetValueMotionProfile outputEnable;
	
	public int timeDurMs;
	
	public int profileSlotSelect1;
}
