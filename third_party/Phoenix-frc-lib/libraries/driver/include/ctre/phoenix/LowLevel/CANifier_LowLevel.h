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

#include "Device_LowLevel.h"
#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/CANifierControlFrame.h"
#include "ctre/phoenix/CANifierFaults.h"
#include "ctre/phoenix/CANifierStatusFrame.h"
#include "ctre/phoenix/CANifierStickyFaults.h"
#include "ctre/phoenix/CANifierVelocityMeasPeriod.h"
#include <FRC_NetworkCommunication/CANSessionMux.h>  //CAN Comm
#include <map>

/** 
 * CANifier Class.
 * Class supports communicating over CANbus.
 */
class LowLevelCANifier : public Device_LowLevel {

public:
	enum GeneralPin{
		QUAD_IDX = 0,
		QUAD_B = 1,
		QUAD_A = 2,
		LIMR = 3,
		LIMF = 4,
		SDA = 5,
		SCL = 6,
		SPI_CS = 7,
		SPI_MISO_PWM2P = 8,
		SPI_MOSI_PWM1P = 9,
		SPI_CLK_PWM0P = 10,
	};

	explicit LowLevelCANifier(int deviceNumber = 0);

	ctre::phoenix::ErrorCode SetLEDOutput( int  dutyCycle,  int  ledChannel);
	ctre::phoenix::ErrorCode SetGeneralOutputs( int  outputsBits,  int  isOutputBits);
	ctre::phoenix::ErrorCode SetGeneralOutput(GeneralPin outputPin, bool bOutputValue, bool bOutputEnable);
	ctre::phoenix::ErrorCode SetPWMOutput( int  pwmChannel,  int  dutyCycle);
	ctre::phoenix::ErrorCode EnablePWMOutput( int  pwmChannel,  bool  bEnable);
	ctre::phoenix::ErrorCode GetGeneralInputs(bool allPins[], uint32_t capacity);
	ctre::phoenix::ErrorCode GetGeneralInput(GeneralPin inputPin, bool * measuredInput);
	ctre::phoenix::ErrorCode GetPWMInput( int  pwmChannel,  double dutyCycleAndPeriod[2]);
	ctre::phoenix::ErrorCode GetLastError();
	ctre::phoenix::ErrorCode GetBatteryVoltage(double * batteryVoltage);
	ctre::phoenix::ErrorCode SetLastError(ctre::phoenix::ErrorCode error);
	ctre::phoenix::ErrorCode GetQuadraturePosition(int * pos);
	ctre::phoenix::ErrorCode SetQuadraturePosition(int newPosition, int timeoutMs);
	ctre::phoenix::ErrorCode GetQuadratureVelocity(int * vel);
	ctre::phoenix::ErrorCode GetQuadratureSensor(int * pos, int * vel);
	ctre::phoenix::ErrorCode ConfigVelocityMeasurementPeriod(
			ctre::phoenix::CANifierVelocityMeasPeriod period, int timeoutMs);
	ctre::phoenix::ErrorCode ConfigVelocityMeasurementWindow(int windowSize, int timeoutMs);

	ctre::phoenix::ErrorCode GetFaults(ctre::phoenix::CANifierFaults & toFill);
	ctre::phoenix::ErrorCode GetStickyFaults(ctre::phoenix::CANifierStickyFaults & toFill) ;
	ctre::phoenix::ErrorCode ClearStickyFaults(int timeoutMs) ;
	ctre::phoenix::ErrorCode SetStatusFramePeriod(ctre::phoenix::CANifierStatusFrame frame, int periodMs, int timeoutMs) ;
	ctre::phoenix::ErrorCode GetStatusFramePeriod(ctre::phoenix::CANifierStatusFrame frame,
			int & periodMs, int timeoutMs) ;
	ctre::phoenix::ErrorCode SetControlFramePeriod(ctre::phoenix::CANifierControlFrame frame,
			int periodMs);

	const static int kMinFirmwareVersionMajor = 0;
	const static int kMinFirmwareVersionMinor = 40;

private:

	static const int kDefaultControlPeriodMs = 20;
	static const int kDefaultPwmOutputPeriodMs = 20;

	bool _SendingPwmOutput = false;

    uint32_t _regInput = 0; //!< Decoded inputs
    uint32_t _regLat = 0; //!< Decoded output latch
    uint32_t _regIsOutput = 0; //!< Decoded data direction register

	ctre::phoenix::ErrorCode _lastError = ctre::phoenix::OKAY;

	void CheckFirmVers(int minMajor = kMinFirmwareVersionMajor, int minMinor = kMinFirmwareVersionMinor, 
							ctre::phoenix::ErrorCode failCode = ctre::phoenix::ErrorCode::FirmwareTooOld);
	void EnsurePwmOutputFrameIsTransmitting();
	void EnableFirmStatusFrame(bool enable);
};

