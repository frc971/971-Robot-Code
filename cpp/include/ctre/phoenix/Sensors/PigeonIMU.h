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

#pragma once

#ifndef CTR_EXCLUDE_WPILIB_CLASSES
#include <string>
#include "ctre/phoenix/LowLevel/CANBusAddressable.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/Sensors/PigeonIMU_ControlFrame.h"
#include "ctre/phoenix/Sensors/PigeonIMU_Faults.h"
#include "ctre/phoenix/Sensors/PigeonIMU_StatusFrame.h"
#include "ctre/phoenix/Sensors/PigeonIMU_StickyFaults.h"

/* forward prototype */
namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {
class TalonSRX;
}
}
}
}

namespace ctre {
namespace phoenix {
namespace sensors {
/**
 * Pigeon IMU Class.
 * Class supports communicating over CANbus and over ribbon-cable (CAN Talon SRX).
 */
class PigeonIMU: public CANBusAddressable {
public:
	/** Data object for holding fusion information. */
	struct FusionStatus {
		double heading;
		bool bIsValid;
		bool bIsFusing;
		std::string description;
		/**
		 * Same as GetLastError()
		 */
		int lastError;
	};
	/** Various calibration modes supported by Pigeon. */
	enum CalibrationMode {
		BootTareGyroAccel = 0,
		Temperature = 1,
		Magnetometer12Pt = 2,
		Magnetometer360 = 3,
		Accelerometer = 5,
	};
	/** Overall state of the Pigeon. */
	enum PigeonState {
		NoComm, Initializing, Ready, UserCalibration,
	};
	/**
	 * Data object for status on current calibration and general status.
	 *
	 * Pigeon has many calibration modes supported for a variety of uses.
	 * The modes generally collects and saves persistently information that makes
	 * the Pigeon signals more accurate.  This includes collecting temperature, gyro, accelerometer,
	 * and compass information.
	 *
	 * For FRC use-cases, typically compass and temperature calibration is not required.
	 *
	 * Additionally when motion driver software in the Pigeon boots, it will perform a fast boot calibration
	 * to initially bias gyro and setup accelerometer.
	 *
	 * These modes can be enabled with the EnterCalibration mode.
	 *
	 * When a calibration mode is entered, caller can expect...
	 *
	 *  - PigeonState to reset to Initializing and bCalIsBooting is set to true.  Pigeon LEDs will blink the boot pattern.
	 *  	This is similar to the normal boot cal, however it can an additional ~30 seconds since calibration generally
	 *  	requires more information.
	 *  	currentMode will reflect the user's selected calibration mode.
	 *
	 *  - PigeonState will eventually settle to UserCalibration and Pigeon LEDs will show cal specific blink patterns.
	 *  	bCalIsBooting is now false.
	 *
	 *  - Follow the instructions in the Pigeon User Manual to meet the calibration specific requirements.
	 * 		When finished calibrationError will update with the result.
	 * 		Pigeon will solid-fill LEDs with red (for failure) or green (for success) for ~5 seconds.
	 * 		Pigeon then perform boot-cal to cleanly apply the newly saved calibration data.
	 */
	struct GeneralStatus {
		/**
		 * The current state of the motion driver.  This reflects if the sensor signals are accurate.
		 * Most calibration modes will force Pigeon to reinit the motion driver.
		 */
		PigeonIMU::PigeonState state;
		/**
		 * The currently applied calibration mode if state is in UserCalibration or if bCalIsBooting is true.
		 * Otherwise it holds the last selected calibration mode (when calibrationError was updated).
		 */
		PigeonIMU::CalibrationMode currentMode;
		/**
		 * The error code for the last calibration mode.
		 * Zero represents a successful cal (with solid green LEDs at end of cal)
		 * and nonzero is a failed calibration (with solid red LEDs at end of cal).
		 * Different calibration
		 */
		int calibrationError;
		/**
		 * After caller requests a calibration mode, pigeon will perform a boot-cal before
		 * entering the requested mode.  During this period, this flag is set to true.
		 */
		bool bCalIsBooting;
		/**
		 * general string description of current status
		 */
		std::string description;
		/**
		 * Temperature in Celsius
		 */
		double tempC;
		/**
		 * Number of seconds Pigeon has been up (since boot).
		 * This register is reset on power boot or processor reset.
		 * Register is capped at 255 seconds with no wrap around.
		 */
		int upTimeSec;
		/**
		 * Number of times the Pigeon has automatically rebiased the gyro.
		 * This counter overflows from 15 -> 0 with no cap.
		 */
		int noMotionBiasCount;
		/**
		 * Number of times the Pigeon has temperature compensated the various signals.
		 * This counter overflows from 15 -> 0 with no cap.
		 */
		int tempCompensationCount;
		/**
		 * Same as GetLastError()
		 */
		int lastError;
	};

	PigeonIMU(int deviceNumber);
	PigeonIMU(ctre::phoenix::motorcontrol::can::TalonSRX * talonSrx);

	int SetYaw(double angleDeg, int timeoutMs);
	int AddYaw(double angleDeg, int timeoutMs);
	int SetYawToCompass(int timeoutMs);

	int SetFusedHeading(double angleDeg, int timeoutMs);
	int AddFusedHeading(double angleDeg, int timeoutMs);
	int SetFusedHeadingToCompass(int timeoutMs);
	int SetAccumZAngle(double angleDeg, int timeoutMs);

	int ConfigTemperatureCompensationEnable(bool bTempCompEnable,
			int timeoutMs);

	int SetCompassDeclination(double angleDegOffset, int timeoutMs);
	int SetCompassAngle(double angleDeg, int timeoutMs);

	int EnterCalibrationMode(CalibrationMode calMode, int timeoutMs);
	int GetGeneralStatus(PigeonIMU::GeneralStatus & genStatusToFill);
	int GetLastError();
	int Get6dQuaternion(double wxyz[4]);
	int GetYawPitchRoll(double ypr[3]);
	int GetAccumGyro(double xyz_deg[3]);
	double GetAbsoluteCompassHeading();
	double GetCompassHeading();
	double GetCompassFieldStrength();
	double GetTemp();
	PigeonState GetState();
	uint32_t GetUpTime();
	int GetRawMagnetometer(int16_t rm_xyz[3]);

	int GetBiasedMagnetometer(int16_t bm_xyz[3]);
	int GetBiasedAccelerometer(int16_t ba_xyz[3]);
	int GetRawGyro(double xyz_dps[3]);
	int GetAccelerometerAngles(double tiltAngles[3]);
	/**
	 * @param status 	object reference to fill with fusion status flags.
	 * @return The fused heading in degrees.
	 */
	double GetFusedHeading(FusionStatus & status);
	/**
	 * @return The fused heading in degrees.
	 */
	double GetFusedHeading();
	uint32_t GetResetCount();
	uint32_t GetResetFlags();
	uint32_t GetFirmVers();

	bool HasResetOccurred();

	static std::string ToString(PigeonIMU::PigeonState state);
	static std::string ToString(CalibrationMode cm);

	ErrorCode ConfigSetCustomParam(int newValue, int paramIndex, int timeoutMs);
	int ConfigGetCustomParam(int paramIndex, int timeoutMs);
	ErrorCode ConfigSetParameter(ParamEnum param, double value,
			uint8_t subValue, int ordinal, int timeoutMs);
	double ConfigGetParameter(ctre::phoenix::ParamEnum param, int ordinal, int timeoutMs);

	ErrorCode SetStatusFramePeriod(PigeonIMU_StatusFrame statusFrame, int periodMs,
			int timeoutMs);

	/**
	 * Gets the period of the given status frame.
	 *
	 * @param frame
	 *            Frame to get the period of.
	 * @param timeoutMs
	 *            Timeout value in ms. @see #ConfigOpenLoopRamp
	 * @return Period of the given status frame.
	 */
	int GetStatusFramePeriod(PigeonIMU_StatusFrame frame,
			int timeoutMs) ;
	ErrorCode SetControlFramePeriod(PigeonIMU_ControlFrame frame,
			int periodMs);
	int GetFirmwareVersion() ;
	ErrorCode GetFaults(PigeonIMU_Faults & toFill) ;
	ErrorCode GetStickyFaults(PigeonIMU_StickyFaults & toFill);
	ErrorCode ClearStickyFaults(int timeoutMs);




	void* GetLowLevelHandle() {
		return _handle;
	}
private:
	/** firmware state reported over CAN */
	enum MotionDriverState {
		Init0 = 0,
		WaitForPowerOff = 1,
		ConfigAg = 2,
		SelfTestAg = 3,
		StartDMP = 4,
		ConfigCompass_0 = 5,
		ConfigCompass_1 = 6,
		ConfigCompass_2 = 7,
		ConfigCompass_3 = 8,
		ConfigCompass_4 = 9,
		ConfigCompass_5 = 10,
		SelfTestCompass = 11,
		WaitForGyroStable = 12,
		AdditionalAccelAdjust = 13,
		Idle = 14,
		Calibration = 15,
		LedInstrum = 16,
		Error = 31,
	};
	/** sub command for the various Set param enums */
	enum TareType {
		SetValue = 0x00, AddOffset = 0x01, MatchCompass = 0x02, SetOffset = 0xFF,
	};
	/** data storage for reset signals */
	struct ResetStats {
		int32_t resetCount;
		int32_t resetFlags;
		int32_t firmVers;
		bool hasReset;
	};
	ResetStats _resetStats = { 0, 0, 0, false };

	/** Portion of the arbID for all status and control frames. */
	void* _handle;
	uint32_t _deviceNumber;
	uint32_t _usageHist = 0;
	uint64_t _cache;
	uint32_t _len;

	/** overall threshold for when frame data is too old */
	const uint32_t EXPECTED_RESPONSE_TIMEOUT_MS = (200);

	int PrivateSetParameter(ParamEnum paramEnum, TareType tareType,
			double angleDeg, int timeoutMs);

	PigeonIMU::PigeonState GetState(int errCode, const uint64_t & statusFrame);
	double GetTemp(const uint64_t & statusFrame);
};
} // namespace signals
} // namespace phoenix
} // namespace ctre
#endif // CTR_EXCLUDE_WPILIB_CLASSES
