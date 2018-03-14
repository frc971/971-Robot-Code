package com.ctre.phoenix.motorcontrol;

public enum RemoteFeedbackDevice {
	None(-1),

	SensorSum(9),
	SensorDifference(10),
	RemoteSensor0(11),
	RemoteSensor1(12),
	SoftwareEmulatedSensor(15);
	
	public final int value;
	RemoteFeedbackDevice(int initValue)
	{
		this.value = initValue;
	}
};