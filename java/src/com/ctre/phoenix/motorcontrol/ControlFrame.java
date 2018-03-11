package com.ctre.phoenix.motorcontrol;

public enum ControlFrame
{
	Control_3_General(0x040080),
	Control_4_Advanced(0x0400C0),
	Control_6_MotProfAddTrajPoint(0x040140);
	
	public final int value;
	ControlFrame(int initValue)
	{
		this.value = initValue;
	}
};
