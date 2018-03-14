/*
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files ( *.crf) and Software
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
package com.ctre.phoenix.sensors;
import com.ctre.phoenix.CTREJNIWrapper;


public class PigeonImuJNI extends CTREJNIWrapper {
  
	public static native long JNI_new_PigeonImu_Talon(int talonID);

	public static native long JNI_new_PigeonImu(int deviceNumber);

	public static native int JNI_ConfigSetCustomParam(long handle, int newValue, int paramIndex, int timeoutMs);

	public static native int JNI_ConfigGetCustomParam(long handle, int paramIndex, int timoutMs);

	public static native int JNI_ConfigSetParameter(long handle, int param, double value, int subValue, int ordinal,
			int timeoutMs);

	public static native double JNI_ConfigGetParameter(long handle, int param, int ordinal, int timeoutMs);

	public static native int JNI_SetStatusFramePeriod(long handle, int statusFrame, int periodMs, int timeoutMs);

	public static native int JNI_SetYaw(long handle, double angleDeg, int timeoutMs);

	public static native int JNI_AddYaw(long handle, double angleDeg, int timeoutMs);
	
	public static native int JNI_SetYawToCompass(long handle, int timeoutMs);
	
	public static native int JNI_SetFusedHeading(long handle, double angleDeg, int timeoutMs);
	
	public static native int JNI_AddFusedHeading(long handle, double angleDeg, int timeoutMs);
	
	public static native int JNI_SetFusedHeadingToCompass(long handle, int timeoutMs);
	
	public static native int JNI_SetAccumZAngle(long handle, double angleDeg, int timeoutMs);
	
	public static native int JNI_ConfigTemperatureCompensationEnable(long handle, int bTempCompEnable, int timeoutMs);
	
	public static native int JNI_SetCompassDeclination(long handle, double angleDegOffset, int timeoutMs);
	
	public static native int JNI_SetCompassAngle(long handle, double angleDeg, int timeoutMs);
	
	public static native int JNI_EnterCalibrationMode(long handle, int calMode, int timeoutMs);
	  
	public static native int JNI_GetGeneralStatus(long handle, double [] params);
	
	public static native int JNI_Get6dQuaternion(long handle, double [] wxyz );
	
	public static native int JNI_GetYawPitchRoll(long handle, double [] ypr);
	
	public static native int JNI_GetAccumGyro(long handle, double [] xyz_deg);
	
	public static native double JNI_GetAbsoluteCompassHeading(long handle);
	
	public static native double JNI_GetCompassHeading(long handle);
	
	public static native double JNI_GetCompassFieldStrength(long handle);
	
	public static native double JNI_GetTemp(long handle);
	
	public static native int JNI_GetUpTime(long handle);
	
	public static native int JNI_GetRawMagnetometer(long handle, short [] rm_xyz);
	
	public static native int JNI_GetBiasedMagnetometer(long handle, short [] bm_xyz);
	
	public static native int JNI_GetBiasedAccelerometer(long handle, short [] ba_xyz);
	
	public static native int JNI_GetRawGyro(long handle, double [] xyz_dps);
	
	public static native int JNI_GetAccelerometerAngles(long handle, double [] tiltAngles);
	  
	public static native int JNI_GetFusedHeading(long handle, double [] params);
	  
	public static native int JNI_GetState(long handle);
	
	public static native int JNI_GetResetCount(long handle);
	
	public static native int JNI_GetResetFlags(long handle);
	
	public static native int JNI_GetFirmwareVersion(long handle);
	  
	public static native int JNI_GetLastError(long handle);
	  
	public static native boolean JNI_HasResetOccurred(long handle);
	
	public static native int JNI_GetStatusFramePeriod(long handle, int frame, int timeoutMs);

	public static native int JNI_SetControlFramePeriod(long handle, int frame, int periodMs);

	public static native int JNI_GetFaults(long handle);

	public static native int JNI_GetStickyFaults(long handle);

	public static native int JNI_ClearStickyFaults(long handle, int timeoutMs);
	
}