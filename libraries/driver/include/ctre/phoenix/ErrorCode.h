#pragma once
#include <stdint.h>

namespace ctre {
namespace phoenix {

enum ErrorCode
: int32_t
{
	OK = 0, 
	OKAY = 0,		//!< No Error - Function executed as expected

	//CAN-Related
	CAN_MSG_STALE = 1,
	CAN_TX_FULL = -1,
	TxFailed = -1,				//!< Could not transmit the CAN frame.
	InvalidParamValue = -2, 	//!< Caller passed an invalid param
	CAN_INVALID_PARAM = -2,
	RxTimeout = -3,	//!< CAN frame has not been received within specified period of time.
	CAN_MSG_NOT_FOUND = -3,
	TxTimeout = -4,				//!< Not used.
	CAN_NO_MORE_TX_JOBS = -4,
	UnexpectedArbId = -5,		//!< Specified CAN Id is invalid.
	CAN_NO_SESSIONS_AVAIL = -5,
	BufferFull = +6,//!< Caller attempted to insert data into a buffer that is full.
	CAN_OVERFLOW = -6,
	SensorNotPresent = -7,		//!< Sensor is not present
	FirmwareTooOld = -8,
	CouldNotChangePeriod = -9,


	//General
	GeneralError = -100,		//!< User Specified General Error
	GENERAL_ERROR = -100,

	//Signal
	SIG_NOT_UPDATED = -200,
	SigNotUpdated = -200,	//!< Have not received an value response for signal.
	NotAllPIDValuesUpdated = -201,

	//Gadgeteer Port Error Codes
	//These include errors between ports and modules
	GEN_PORT_ERROR = -300,
	PORT_MODULE_TYPE_MISMATCH = -301,

	//Gadgeteer Module Error Codes
	//These apply only to the module units themselves
	GEN_MODULE_ERROR = -400,
	MODULE_NOT_INIT_SET_ERROR = -401,
	MODULE_NOT_INIT_GET_ERROR = -402,

	//API
	WheelRadiusTooSmall = -500,
	TicksPerRevZero = -501,
	DistanceBetweenWheelsTooSmall = -502,
	GainsAreNotSet = -503,

	//Higher Level
	IncompatibleMode = -600,
	InvalidHandle = -601,		//!< Handle does not match stored map of handles
	
	//Firmware Versions
	FeatureRequiresHigherFirm = -700,
	TalonFeatureRequiresHigherFirm = -701,

	//CAN Related
	PulseWidthSensorNotPresent = 10,	//!< Special Code for "isSensorPresent"

	//General
	GeneralWarning = 100,
	FeatureNotSupported = 101, // feature not implement in the API or firmware
	NotImplemented = 102, // feature not implement in the API
	FirmVersionCouldNotBeRetrieved = 103,
	FeaturesNotAvailableYet = 104, // feature will be release in an upcoming release
	ControlModeNotValid = 105, // Current control mode of motor controller not valid for this call

	ControlModeNotSupportedYet = 106,
	CascadedPIDNotSupporteYet= 107,
	AuxiliaryPIDNotSupportedYet= 107,
	RemoteSensorsNotSupportedYet= 108,
	MotProfFirmThreshold= 109,
	MotProfFirmThreshold2 = 110,
};

} // namespace phoenix
} // namespace ctre
