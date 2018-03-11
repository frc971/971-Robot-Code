package com.ctre.phoenix;

import java.util.HashMap;

public enum ErrorCode {
	OK(0), 						//!< No Error - Function executed as expected

	//CAN-Related
	CAN_MSG_STALE(1),
	CAN_TX_FULL(-1),
	TxFailed(-1),				//!< Could not transmit the CAN frame.
	InvalidParamValue(-2), 	//!< Caller passed an invalid param
	CAN_INVALID_PARAM(-2),
	RxTimeout(-3),				//!< CAN frame has not been received within specified period of time.
	CAN_MSG_NOT_FOUND(-3),
	TxTimeout(-4),				//!< Not used.
	CAN_NO_MORE_TX_JOBS(-4),
	UnexpectedArbId(-5),		//!< Specified CAN Id is invalid.
	CAN_NO_SESSIONS_AVAIL(-5),
	BufferFull(+6),			//!< Caller attempted to insert data into a buffer that is full.
	CAN_OVERFLOW(-6),
	SensorNotPresent(-7),		//!< Sensor is not present
	FirmwareTooOld (-8),


	//General
	GeneralError(-100),		//!< User Specified General Error
	GENERAL_ERROR(-100),

	//Signal
	SIG_NOT_UPDATED(-200),
	SigNotUpdated(-200),			//!< Have not received an value response for signal.
	NotAllPIDValuesUpdated(-201),

	//Gadgeteer Port Error Codes
	//These include errors between ports and modules
	GEN_PORT_ERROR(-300),
	PORT_MODULE_TYPE_MISMATCH(-301),
	//Gadgeteer Module Error Codes
	//These apply only to the module units themselves
	GEN_MODULE_ERROR(-400),
	MODULE_NOT_INIT_SET_ERROR(-401),
	MODULE_NOT_INIT_GET_ERROR(-402),


	//API
	WheelRadiusTooSmall(-500),
	TicksPerRevZero(-501),
	DistanceBetweenWheelsTooSmall(-502),
	GainsAreNotSet(-503),

	//Higher Level
	IncompatibleMode(-600),
	InvalidHandle(-601),		//!< Handle does not match stored map of handles


	//CAN Related
	PulseWidthSensorNotPresent (10),	//!< Special Code for "isSensorPresent"

	//General
	GeneralWarning(100),
	FeatureNotSupported(101),
	NotImplemented(102),
	FirmVersionCouldNotBeRetrieved (103),
	FeaturesNotAvailableYet(104),
	ControlModeNotValid(105),

	ControlModeNotSupportedYet(106),
	CascadedPIDNotSupportedYet(107),
	AuxiliaryPIDNotSupportedYet(107),
	RemoteSensorsNotSupportedYet(108),
	MotProfFirmThreshold(109),
	MotProfFirmThreshold2(110);

	//---------------------- Integral To Enum operators -----------//
    public final int value; //!< Hold the integral value of an enum instance.
    /** Keep singleton map to quickly lookup enum via int */
    private static HashMap<Integer, ErrorCode> _map = null;
    /** private c'tor for above declarations */
	private ErrorCode(int initValue) {this.value = initValue;	}
	/** static c'tor, prepare the map */
    static {
    	_map = new HashMap<Integer, ErrorCode>();
		for (ErrorCode type : ErrorCode.values()) {
			_map.put(type.value, type);
		}
    }
    /** public lookup to convert int to enum */
	public static ErrorCode valueOf(int value) {
		ErrorCode retval = _map.get(value);
		if (retval != null)
			return retval;
		return GeneralError;
	}

	/** @return the first nonzero error code */
	public static ErrorCode worstOne(ErrorCode errorCode1, ErrorCode errorCode2) {
		if (errorCode1.value != 0)
			return errorCode1;
		return errorCode2;
	}

	/** @return the first nonzero error code */
	public static ErrorCode worstOne(ErrorCode errorCode1, ErrorCode errorCode2, ErrorCode errorCode3) {
		if (errorCode1.value != 0)
			return errorCode1;
		if (errorCode2.value != 0)
			return errorCode2;
		return errorCode3;
	}

	/** @return the first nonzero error code */
	public static ErrorCode worstOne(ErrorCode errorCode1, ErrorCode errorCode2, ErrorCode errorCode3,
			ErrorCode errorCode4) {
		if (errorCode1.value != 0)
			return errorCode1;
		if (errorCode2.value != 0)
			return errorCode2;
		if (errorCode3.value != 0)
			return errorCode3;
		return errorCode4;
	}
};
