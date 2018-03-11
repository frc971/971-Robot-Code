package com.ctre.phoenix.motorcontrol;

public enum ControlMode
{
	PercentOutput(0),
	Position(1),
	Velocity(2),
	Current(3),
	Follower(5),
	MotionProfile(6),
	MotionMagic(7),
	MotionProfileArc(10),

	Disabled(15);

	public final int value;
	ControlMode(int initValue)
	{
		this.value = initValue;
	}
};
