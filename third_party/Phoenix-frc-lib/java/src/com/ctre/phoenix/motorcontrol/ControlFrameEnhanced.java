package com.ctre.phoenix.motorcontrol;

public enum ControlFrameEnhanced
{
	Control_2_Enable_50m(0x040040),
	Control_3_General(0x040080),
	Control_4_Advanced(0x0400C0),
	Control_5_FeedbackOutputOverride(0x040100),
	Control_6_MotProfAddTrajPoint(0x040140);
	
	public final int value;
	ControlFrameEnhanced(int initValue)
	{
		this.value = initValue;
	}
};