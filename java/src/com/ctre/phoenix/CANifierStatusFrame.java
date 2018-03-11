package com.ctre.phoenix;

public enum CANifierStatusFrame {
	Status_1_General(0x041400), 
	Status_2_General(0x041440), 
	Status_3_PwmInputs0(0x041480), 
	Status_4_PwmInputs1(0x0414C0), 
	Status_5_PwmInputs2(0x041500), 
	Status_6_PwmInputs3(0x041540), 
	Status_8_Misc(0x0415C0);

	public static CANifierStatusFrame valueOf(int value) {
		for (CANifierStatusFrame frame : values()) {
			if (frame.value == value) {
				return frame;
			}
		}
		return null;
	}

	public final int value;

	CANifierStatusFrame(int initValue) {
		this.value = initValue;
	}
}