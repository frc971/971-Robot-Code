/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#ifndef _SERIALPORT_H
#define _SERIALPORT_H

#include "SerialPort.h"

void OpenSerialPort(UINT32 baudRate, UINT8 dataBits, SerialPort::Parity parity, SerialPort::StopBits stopBits);
void SetSerialFlowControl(SerialPort::FlowControl flowControl);
void EnableSerialTermination(char terminator);
void DisableSerialTermination();
INT32 GetSerialBytesReceived();
void PrintfSerial(const char *writeFmt, ...);
void ScanfSerial(const char *readFmt, ...);
UINT32 ReadSerialPort(char *buffer, INT32 count);
UINT32 WriteSerialPort(const char *buffer, INT32 count);
void SetSerialTimeout(INT32 timeout_ms);
void SetSerialWriteBufferMode(SerialPort::WriteBufferMode mode);
void FlushSerialPort();
void ResetSerialPort();

#endif
