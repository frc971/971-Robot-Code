#pragma once

#include "ctre/phoenix/ErrorCode.h"
#include <set>

namespace CANifier_CCI{
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
}

extern "C"{
	void *c_CANifier_Create1(int deviceNumber);
	ctre::phoenix::ErrorCode c_CANifier_GetDescription(void *handle, char * toFill, int toFillByteSz, int * numBytesFilled);
	ctre::phoenix::ErrorCode c_CANifier_SetLEDOutput(void *handle,  uint32_t  dutyCycle,  uint32_t  ledChannel);
	ctre::phoenix::ErrorCode c_CANifier_SetGeneralOutputs(void *handle,  uint32_t  outputsBits,  uint32_t  isOutputBits);
	ctre::phoenix::ErrorCode c_CANifier_SetGeneralOutput(void *handle,  uint32_t  outputPin,  bool  outputValue,  bool  outputEnable);
	ctre::phoenix::ErrorCode c_CANifier_SetPWMOutput(void *handle,  uint32_t  pwmChannel,  uint32_t  dutyCycle);
	ctre::phoenix::ErrorCode c_CANifier_EnablePWMOutput(void *handle, uint32_t pwmChannel, bool bEnable);
	ctre::phoenix::ErrorCode c_CANifier_GetGeneralInputs(void *handle, bool allPins[], uint32_t capacity);
	ctre::phoenix::ErrorCode c_CANifier_GetGeneralInput(void *handle, uint32_t inputPin, bool * measuredInput);
	ctre::phoenix::ErrorCode c_CANifier_GetPWMInput(void *handle,  uint32_t  pwmChannel,  double dutyCycleAndPeriod [2]);
	ctre::phoenix::ErrorCode c_CANifier_GetLastError(void *handle);
	ctre::phoenix::ErrorCode c_CANifier_GetBusVoltage(void *handle, double * batteryVoltage);
	ctre::phoenix::ErrorCode c_CANifier_GetQuadraturePosition(void *handle, int * pos);
	ctre::phoenix::ErrorCode c_CANifier_SetQuadraturePosition(void *handle, int pos, int timeoutMs);
	ctre::phoenix::ErrorCode c_CANifier_GetQuadratureVelocity(void *handle, int * vel);
	ctre::phoenix::ErrorCode c_CANifier_GetQuadratureSensor(void *handle, int * pos, int * vel);
	ctre::phoenix::ErrorCode c_CANifier_ConfigVelocityMeasurementPeriod(void *handle, int period, int timeoutMs);
	ctre::phoenix::ErrorCode c_CANifier_ConfigVelocityMeasurementWindow(void *handle, int window, int timeoutMs);
	void c_CANifier_SetLastError(void *handle, int error);
	ctre::phoenix::ErrorCode c_CANifier_ConfigSetParameter(void *handle, int param, double value, int subValue, int ordinal, int timeoutMs);
	ctre::phoenix::ErrorCode c_CANifier_ConfigGetParameter(void *handle, int param, double *value, int ordinal, int timeoutMs);
	ctre::phoenix::ErrorCode c_CANifier_ConfigSetCustomParam(void *handle, int newValue, int paramIndex, int timeoutMs);
	ctre::phoenix::ErrorCode c_CANifier_ConfigGetCustomParam(void *handle, int *readValue, int paramIndex, int timoutMs);
	ctre::phoenix::ErrorCode c_CANifier_GetFaults(void *handle, int * param) ;
	ctre::phoenix::ErrorCode c_CANifier_GetStickyFaults(void *handle, int * param) ;
	ctre::phoenix::ErrorCode c_CANifier_ClearStickyFaults(void *handle, int timeoutMs) ;
	ctre::phoenix::ErrorCode c_CANifier_GetFirmwareVersion(void *handle, int *firmwareVers);
	ctre::phoenix::ErrorCode c_CANifier_HasResetOccurred(void *handle, bool * hasReset);
	ctre::phoenix::ErrorCode c_CANifier_SetStatusFramePeriod(void *handle, int frame, int periodMs, int timeoutMs);
	ctre::phoenix::ErrorCode c_CANifier_GetStatusFramePeriod(void *handle, int frame, int *periodMs, int timeoutMs);
	ctre::phoenix::ErrorCode c_CANifier_SetControlFramePeriod(void *handle, int frame,	int periodMs) ;
}
