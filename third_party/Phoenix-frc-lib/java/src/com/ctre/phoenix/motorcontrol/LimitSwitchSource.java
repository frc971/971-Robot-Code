package com.ctre.phoenix.motorcontrol;

public enum LimitSwitchSource {
	FeedbackConnector(0), RemoteTalonSRX(1), RemoteCANifier(2), Deactivated(3);

	public int value;

	LimitSwitchSource(int value) {
		this.value = value;
	}
};