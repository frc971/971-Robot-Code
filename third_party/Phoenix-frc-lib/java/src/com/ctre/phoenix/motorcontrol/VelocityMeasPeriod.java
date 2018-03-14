package com.ctre.phoenix.motorcontrol;

public enum VelocityMeasPeriod {
	Period_1Ms(1),
	Period_2Ms(2),
	Period_5Ms(5),
	Period_10Ms(10),
	Period_20Ms(20),
	Period_25Ms(25),
	Period_50Ms(50),
	Period_100Ms(100);
	
	public int value;
	VelocityMeasPeriod(int value)
	{
		this.value = value;
	}
};