package com.ctre.phoenix.motorcontrol;

public enum RemoteLimitSwitchSource {
	RemoteTalonSRX(1), RemoteCANifier(2), Deactivated(3);

	public int value;

	RemoteLimitSwitchSource(int value) {
		this.value = value;
	}
};