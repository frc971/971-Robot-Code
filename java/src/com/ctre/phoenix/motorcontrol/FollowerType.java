package com.ctre.phoenix.motorcontrol;

public enum FollowerType {
  PercentOutput(0),
  AuxOutput1(1);

	public int value;
	FollowerType(int value)
	{
		this.value = value;
	}
};
