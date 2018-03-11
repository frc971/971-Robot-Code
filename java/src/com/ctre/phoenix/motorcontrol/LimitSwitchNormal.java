package com.ctre.phoenix.motorcontrol;

public enum LimitSwitchNormal {
	NormallyOpen(0), NormallyClosed(1), Disabled(2);

	public int value;

	LimitSwitchNormal(int value) {
		this.value = value;
	}
};