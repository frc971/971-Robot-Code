/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _COMPRESSOR_H
#define _COMPRESSOR_H

void CreateCompressor(UINT32 pressureSwitch, UINT32 relayChannel);
void CreateCompressor(UINT32 pressureSwitchSlot, UINT32 pressureSwitchChannel,
						UINT32 relaySlot, UINT32 relayChannel);
void StartCompressor();
void StopCompressor();
bool CompressorEnabled();

void DeleteCompressor();
#endif

