package com.ctre.phoenix.motorcontrol;

public enum StatusFrame {
	Status_1_General(0x1400),
	Status_2_Feedback0(0x1440),
	Status_4_AinTempVbat(0x14C0),
	Status_6_Misc(0x1540),
	Status_7_CommStatus(0x1580),
	Status_9_MotProfBuffer(0x1600),
	/**
	 * Old name for Status 10 Frame.
	 * Use Status_10_Targets instead.
	 */
	Status_10_MotionMagic(0x1640),
	/**
	 * Correct name for Status 10 Frame.
	 * Functionally equivalent to Status_10_MotionMagic
	 */
	Status_10_Targets(0x1640),
	Status_12_Feedback1(0x16C0),
	Status_13_Base_PIDF0(0x1700),
	Status_14_Turn_PIDF1(0x1740),
	Status_15_FirmwareApiStatus(0x1780);

	public int value;
	StatusFrame(int value)
	{
		this.value = value;
	}
};
