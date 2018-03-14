package com.ctre.phoenix;

public enum CANifierControlFrame {
	Control_1_General(0x040000), 
	Control_2_PwmOutput(0x040040);

	public static CANifierControlFrame valueOf(int value) {
		for (CANifierControlFrame frame : values()) {
			if (frame.value == value) {
				return frame;
			}
		}
		return null;
	}

	public final int value;

	CANifierControlFrame(int initValue) {
		this.value = initValue;
	}
}
