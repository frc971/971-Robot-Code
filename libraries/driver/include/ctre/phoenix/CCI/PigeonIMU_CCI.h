/*
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
 * API Libraries ONLY when in use with Cross The Road Electronics hardware products.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
 
 #pragma once
 
#include "ctre/phoenix/ErrorCode.h"
 #include <map>
 
 static std::map<void *, bool> pigeonPresent;
 
 extern "C"{
	void *c_PigeonIMU_Create2(int talonDeviceID);
	void *c_PigeonIMU_Create1(int deviceNumber);
	// void c_PigeonIMU_Destroy(void *handle);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetDescription(void *handle, char * toFill, int toFillByteSz, int * numBytesFilled);
	ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetParameter(void *handle, int param, double paramValue, int subValue, int ordinal, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetParameter(void *handle, int param, double *value, int ordinal, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetCustomParam(void *handle, int newValue, int paramIndex, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetCustomParam(void *handle, int *readValue, int paramIndex, int timoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_SetYaw(void *handle, double angleDeg, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_AddYaw(void *handle, double angleDeg, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_SetYawToCompass(void *handle, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_SetFusedHeading(void *handle, double angleDeg, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_AddFusedHeading(void *handle, double angleDeg, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_SetFusedHeadingToCompass(void *handle, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_SetAccumZAngle(void *handle, double angleDeg, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_ConfigTemperatureCompensationEnable(void *handle, int bTempCompEnable, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_SetCompassDeclination(void *handle, double angleDegOffset, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_SetCompassAngle(void *handle, double angleDeg, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_EnterCalibrationMode(void *handle, int calMode, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetGeneralStatus(void *handle, int *state, int *currentMode, int *calibrationError, int *bCalIsBooting, double *tempC, int *upTimeSec, int *noMotionBiasCount, int *tempCompensationCount, int *lastError);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetLastError(void *handle);
	ctre::phoenix::ErrorCode c_PigeonIMU_Get6dQuaternion(void *handle, double wxyz[4]);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetYawPitchRoll(void *handle, double ypr[3]);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetAccumGyro(void *handle, double xyz_deg[3]);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetAbsoluteCompassHeading(void *handle, double *value);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetCompassHeading(void *handle, double *value);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetCompassFieldStrength(void *handle, double *value);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetTemp(void *handle, double *value);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetState(void *handle, int *state);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetUpTime(void *handle, int *value);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetRawMagnetometer(void *handle, short rm_xyz[3]);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetBiasedMagnetometer(void *handle, short bm_xyz[3]);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetBiasedAccelerometer(void *handle, short ba_xyz[3]);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetRawGyro(void *handle, double xyz_dps[3]);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetAccelerometerAngles(void *handle, double tiltAngles[3]);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetFusedHeading2(void *handle, int *bIsFusing, int *bIsValid, double *value, int *lastError);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetFusedHeading1(void *handle, double *value);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetResetCount(void *handle, int *value);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetResetFlags(void *handle, int *value);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetFirmwareVersion(void *handle, int * firmwareVers);
	ctre::phoenix::ErrorCode c_PigeonIMU_HasResetOccurred(void *handle, bool * hasReset);
	ctre::phoenix::ErrorCode c_PigeonIMU_SetLastError(void *handle, int value);
	ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetCustomParam(void *handle, int newValue, int paramIndex, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetCustomParam(void *handle, int *readValue, int paramIndex, int timoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetParameter(void *handle, int param, double value, int subValue, int ordinal, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetParameter(void *handle, int param, double *value, int ordinal, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetFaults(void *handle, int * param);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetStickyFaults(void *handle, int * param);
	ctre::phoenix::ErrorCode c_PigeonIMU_ClearStickyFaults(void *handle, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_SetStatusFramePeriod(void *handle, int frame, int periodMs, int timeoutMs);
	ctre::phoenix::ErrorCode c_PigeonIMU_GetStatusFramePeriod(void *handle, int frame, int *periodMs, int timeoutMs) ;
	ctre::phoenix::ErrorCode c_PigeonIMU_SetControlFramePeriod(void *handle, int frame, int periodMs) ;
}
