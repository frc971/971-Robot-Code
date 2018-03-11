package com.ctre.phoenix.motorcontrol;

public enum NeutralMode {
	/** Use the NeutralMode that is set by the jumper wire on the CAN device */
	EEPROMSetting(0),
	/** Do not attempt to stop the motor. Instead allow it to coast to a stop
	 without applying resistance. */
	Coast(1),
	/** Stop the motor's rotation by applying a force. */
	Brake(2);
	
	public int value;
	NeutralMode(int value)
	{
		this.value = value;
	}
};