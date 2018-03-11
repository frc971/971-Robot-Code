package com.ctre.phoenix.motion;

public enum SetValueMotionProfile {
	Invalid(-1), Disable(0), Enable(1), Hold(2);

	public final int value;

	SetValueMotionProfile(int initValue) {
		this.value = initValue;
	}
	
	public static SetValueMotionProfile valueOf(int value) {
		for (SetValueMotionProfile e : SetValueMotionProfile.values()) {
			if (e.value == value) {
				return e;
			}
		}
		return Invalid;
	}
}
