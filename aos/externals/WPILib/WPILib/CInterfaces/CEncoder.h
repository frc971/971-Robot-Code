/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef _C_ENCODER_H
#define _C_ENCODER_H

void StartEncoder(UINT32 aChannel, UINT32 bChannel);
INT32 GetEncoder(UINT32 aChannel, UINT32 bChannel);
void ResetEncoder(UINT32 aChannel, UINT32 bChannel);
void StopEncoder(UINT32 aChannel, UINT32 bChannel);
double GetEncoderPeriod(UINT32 aChannel, UINT32 bChannel);
void SetMaxEncoderPeriod(UINT32 aChannel, UINT32 bChannel, double maxPeriod);
bool GetEncoderStopped(UINT32 aChannel, UINT32 bChannel);
bool GetEncoderDirection(UINT32 aChannel, UINT32 bChannel);
double GetEncoderDistance(UINT32 aChannel, UINT32 bChannel);
double GetEncoderRate(UINT32 aChannel, UINT32 bChannel);
void SetMinEncoderRate(UINT32 aChannel, UINT32 bChannel, double minRate);
void SetEncoderDistancePerPulse(UINT32 aChannel, UINT32 bChannel, double distancePerPulse);
void SetEncoderReverseDirection(UINT32 aChannel, UINT32 bChannel, bool reversedDirection);
void StartEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);
INT32 GetEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);
void ResetEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);
void StopEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);
double GetEncoderPeriod(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);
void SetMaxEncoderPeriod(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel, double maxPeriod);
bool GetEncoderStopped(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);
bool GetEncoderDirection(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);
double GetEncoderDistance(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);
double GetEncoderRate(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);
void SetMinEncoderRate(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel, double minRate);
void SetEncoderDistancePerPulse(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel, double distancePerPulse);
void SetEncoderReverseDirection(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel, bool reversedDirection);
void DeleteEncoder(UINT32 aChannel, UINT32 bChannel);
void DeleteEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);

typedef void *EncoderObject;

EncoderObject CreateEncoder(UINT32 aChannel, UINT32 bChannel);
EncoderObject CreateEncoder(UINT8 amoduleNumber, UINT32 aChannel, UINT8 bmoduleNumber, UINT32 bChannel);
void StartEncoder(EncoderObject o);
INT32 GetEncoder(EncoderObject o);
void ResetEncoder(EncoderObject o);
void StopEncoder(EncoderObject o);
double GetEncoderPeriod(EncoderObject o);
void SetMaxEncoderPeriod(EncoderObject o, double maxPeriod);
bool GetEncoderStopped(EncoderObject o);
bool GetEncoderDirection(EncoderObject o);
double GetEncoderDistance(EncoderObject o);
double GetEncoderRate(EncoderObject o);
void SetMinEncoderRate(EncoderObject o, double minRate);
void SetEncoderDistancePerPulse(EncoderObject o, double distancePerPulse);
void SetEncoderReverseDirection(EncoderObject o, bool reversedDirection);
void DeleteEncoder(EncoderObject o);
#endif
