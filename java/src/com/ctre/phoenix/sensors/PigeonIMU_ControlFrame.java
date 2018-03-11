package com.ctre.phoenix.sensors;

/** Enumerated type for control frame types. */
public enum PigeonIMU_ControlFrame {
	Control_1(0x00042800);

	public final int value;

	PigeonIMU_ControlFrame(int initValue) {
		this.value = initValue;
	}

	public static PigeonIMU_ControlFrame valueOf(int value) {
		for (PigeonIMU_ControlFrame mode : values()) {
			if (mode.value == value) {
				return mode;
			}
		}
		return null;
	}
}
