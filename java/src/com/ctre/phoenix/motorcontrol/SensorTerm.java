package com.ctre.phoenix.motorcontrol;

public enum SensorTerm {
	Sum0(0),
	Sum1(1),
	Diff0(2),
	Diff1(3);
	
	public int value;
	SensorTerm(int value)
	{
		this.value = value;
	}
};