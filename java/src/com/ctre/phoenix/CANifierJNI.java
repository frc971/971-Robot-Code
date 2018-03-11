/*
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
 * API Libraries ONLY when in use with Cross The Road Electronics hardware products.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
 package com.ctre.phoenix;
 
 public class CANifierJNI extends CTREJNIWrapper {
	 
	public enum GeneralPin
	{
		QUAD_IDX(0),
		QUAD_B (1),
		QUAD_A (2),
		LIMR (3),
		LIMF (4),
		SDA (5),
		SCL (6),
		SPI_CS (7),
		SPI_MISO_PWM2P (8),
		SPI_MOSI_PWM1P (9),
		SPI_CLK_PWM0P (10);
		
		final public int value;

		GeneralPin(int value) {
			this.value = value;
		}
	}
	
	public static native long JNI_new_CANifier(int deviceNumber);

	public static native void JNI_SetLEDOutput(long handle, int dutyCycle, int ledChannel);

	public static native void JNI_SetGeneralOutputs(long handle, int outputBits, int isOutputBits);

	public static native void JNI_SetGeneralOutput(long handle, int outputPin, boolean outputValue,
			boolean outputEnable);

	public static native void JNI_SetPWMOutput(long handle, int pwmChannel, int dutyCycle);

	public static native void JNI_EnablePWMOutput(long handle, int pwmChannel, boolean bEnable);

	public static native void JNI_GetGeneralInputs(long handle, boolean[] allPins);

	public static native boolean JNI_GetGeneralInput(long handl, int inputPin);

	public static native void JNI_GetPWMInput(long handle, int pwmChannel, double dutyCycleAndPeriod[]);

	public static native int JNI_GetLastError(long handle);

	public static native double JNI_GetBatteryVoltage(long handle);
	
	public static native int JNI_GetQuadraturePosition(long handle);
	
	public static native int JNI_SetQuadraturePosition(long handle, int newPosition, int timeoutMs);
	
	public static native int JNI_GetQuadratureVelocity(long handle);
	
	public static native int JNI_ConfigVelocityMeasurementPeriod(long handle, int period, int timeoutMs);
	
	public static native int JNI_ConfigVelocityMeasurementWindow(long handle, int windowSize, int timeoutMs);

	public static native int JNI_ConfigSetCustomParam(long handle, int newValue, int paramIndex, int timeoutMs);

	public static native int JNI_ConfigGetCustomParam(long handle, int paramIndex, int timoutMs);

	public static native int JNI_ConfigSetParameter(long handle, int param, double value, int subValue, int ordinal,
			int timeoutMs);

	public static native double JNI_ConfigGetParameter(long handle, int param, int ordinal, int timeoutMs);

	public static native int JNI_SetStatusFramePeriod(long handle, int statusFrame, int periodMs, int timeoutMs);

	public static native int JNI_GetStatusFramePeriod(long handle, int frame, int timeoutMs);

	public static native int JNI_SetControlFramePeriod(long handle, int frame, int periodMs);

	public static native int JNI_GetFirmwareVersion(long handle);

	public static native boolean JNI_HasResetOccurred(long handle);

	public static native int JNI_GetFaults(long handle);

	public static native int JNI_GetStickyFaults(long handle);

	public static native int JNI_ClearStickyFaults(long handle, int timeoutMs);

	public static native double JNI_GetBusVoltage(long handle);
 }