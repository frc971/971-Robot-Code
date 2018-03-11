#pragma once

#ifndef CTR_EXCLUDE_WPILIB_CLASSES

#include <cstdint>
#include "ctre/phoenix/LowLevel/CANBusAddressable.h"
#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/CANifierControlFrame.h"
#include "ctre/phoenix/CANifierStatusFrame.h"
#include "ctre/phoenix/CANifierStickyFaults.h"
#include "ctre/phoenix/CANifierFaults.h"
#include "ctre/phoenix/CANifierVelocityMeasPeriod.h"

namespace ctre {namespace phoenix {
	/**
	 * CTRE CANifier
	 *
	 * Device for interfacing common devices to the CAN bus.
	 */
class CANifier: public CANBusAddressable {
public:
	/**
	 * Enum for the LED Output Channels
	 */
	enum LEDChannel {
		LEDChannelA = 0, LEDChannelB = 1, LEDChannelC = 2,
	};

	/**
	 * Enum for the PWM Input Channels
	 */
	enum PWMChannel {
		PWMChannel0 = 0, PWMChannel1 = 1, PWMChannel2 = 2, PWMChannel3 = 3,
	};
	const int PWMChannelCount = 4;

	/**
	 * General IO Pins on the CANifier
	 */
	enum GeneralPin {
		QUAD_IDX = 0,	//----- Must match CANifier_CCI enums -----//
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

	/**
	 * Structure to hold the pin values.
	 */
	struct PinValues {
		bool QUAD_IDX;
		bool QUAD_B;
		bool QUAD_A;
		bool LIMR;
		bool LIMF;
		bool SDA;
		bool SCL;
		bool SPI_CS_PWM3;
		bool SPI_MISO_PWM2;
		bool SPI_MOSI_PWM1;
		bool SPI_CLK_PWM0;
	};

	CANifier(int deviceNumber);
	ErrorCode SetLEDOutput(double percentOutput, LEDChannel ledChannel);
	ErrorCode SetGeneralOutput(GeneralPin outputPin, bool outputValue, bool outputEnable);
	ErrorCode SetGeneralOutputs(int outputBits, int isOutputBits);
	ErrorCode GetGeneralInputs(PinValues &allPins);
	bool GetGeneralInput(GeneralPin inputPin);
	int GetQuadraturePosition();
	int GetQuadratureVelocity();
	ErrorCode SetQuadraturePosition(int newPosition, int timeoutMs);
	ErrorCode ConfigVelocityMeasurementPeriod(
			CANifierVelocityMeasPeriod period, int timeoutMs);
	ErrorCode ConfigVelocityMeasurementWindow(int windowSize, int timeoutMs);
	/**
	 * Gets the bus voltage seen by the motor controller.
	 *
	 * @return The bus voltage value (in volts).
	 */
	double GetBusVoltage();
	ErrorCode GetLastError();
	ErrorCode SetPWMOutput(int pwmChannel, double dutyCycle);
	ErrorCode EnablePWMOutput(int pwmChannel, bool bEnable);
	ErrorCode GetPWMInput(PWMChannel pwmChannel, double dutyCycleAndPeriod[]);

	//------ Custom Persistent Params ----------//
	ErrorCode ConfigSetCustomParam(int newValue, int paramIndex,
			int timeoutMs);
	int ConfigGetCustomParam(int paramIndex,
			int timeoutMs);
	//------ Generic Param API, typically not used ----------//
	ErrorCode ConfigSetParameter(ParamEnum param, double value,
			uint8_t subValue, int ordinal, int timeoutMs);
	double ConfigGetParameter(ParamEnum param, int ordinal, int timeoutMs);


	ErrorCode SetStatusFramePeriod(CANifierStatusFrame statusFrame,
			int periodMs, int timeoutMs);
	/**
	 * Gets the period of the given status frame.
	 *
	 * @param frame
	 *            Frame to get the period of.
	 * @param timeoutMs
	 *            Timeout value in ms. @see #ConfigOpenLoopRamp
	 * @return Period of the given status frame.
	 */
	int GetStatusFramePeriod(CANifierStatusFrame frame, int timeoutMs);
	ErrorCode SetControlFramePeriod(CANifierControlFrame frame, int periodMs);
	/**
	 * Gets the firmware version of the device.
	 *
	 * @return Firmware version of device.
	 */
	int GetFirmwareVersion();
	/**
	 * Returns true if the device has reset since last call.
	 *
	 * @return Has a Device Reset Occurred?
	 */
	bool HasResetOccurred();
	ErrorCode GetFaults(CANifierFaults & toFill);
	ErrorCode GetStickyFaults(CANifierStickyFaults & toFill);
	ErrorCode ClearStickyFaults(int timeoutMs);

private:
	void* m_handle;
	bool _tempPins[11];
};
}}
#endif // CTR_EXCLUDE_WPILIB_CLASSES
