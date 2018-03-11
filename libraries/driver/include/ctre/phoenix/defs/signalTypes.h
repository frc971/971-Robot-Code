#ifndef signalTypes__h_
#define signalTypes__h_

enum{
	modeDutyCycleControl = 0,	//!< Demand is signed 16bit 0.16fixedPt. 7FFF is fullfor.  8000 is fullRev.
	modePosControl = 1,			//!< Demand 24bit position.
	modeSpeedControl = 2,		//!< Demand is 24 bit speed.
	modeCurrentControl = 3,		//!< Demand is 24 bit current.
	modeSlaveFollower = 5,		//!< Demand is the can node to follow.
	modeMotionProfile = 6,		//!< Demand is unused,could be used in future. Control6 has everything we want.
	modeMotionMagic = 7,		//!< Reserved
	motionMagicArc = 8,
	//9
	motionProfileArc = 10,
	//11
	//12
	//13
#ifdef SUPPORT_ONE_SHOT_CONTROL_MODE
	modeOneShot = 14,
#endif
	modeNoDrive = 15,
};

typedef enum _feedbackDevice_t{
	kQuadEncoder = 0,
	//1
	kAnalog = 2,
	//3
	Tachometer= 4,
	kPulseWidthEncodedPosition = 8,
	
	kSensorSum = 9,
	kSensorDifference = 10,
	kRemoteSensor0 = 11,
	kRemoteSensor1 = 12,
	//13
	//14
	kSoftwarEmulatedSensor=15,
}feedbackDevice_t;

typedef enum _MotProf_OutputType_t {
	MotProf_OutputType_Disabled = 0,
	MotProf_OutputType_Enabled,
	MotProf_OutputType_Hold,
	MotProf_OutputType_Invalid,
}MotProf_OutputType_t;

/**
 * Saved to limitSwitchForward_Source/limitSwitchReverse_Source
 */
typedef enum _LimitSwitchSource_t {
	LSS_Local=0,
	LSS_RemoteTalon=1,
	LSS_RemoteCanif=2,
	LSS_Deactivated=3,
}LimitSwitchSource_t;

/**
 * Saved to limitSwitchForward_normClosedAndDis / limitSwitchReverse_normClosedAndDis
 */
typedef enum _LimitSwitchNormClosedAndDis_t {
	LSNCD_NormallyOpen=0,
	LSNCD_NormallyClosed=1,
	LSNCD_NormallyDisabled=2,
}LimitSwitchNormClosedAndDis_t;

typedef enum _RemoteSensorSource_t {
	RSS_Off,
	RSS_RemoteTalonSelSensor,
	RSS_RemotePigeon_Yaw,
	RSS_RemotePigeon_Pitch,
	RSS_RemotePigeon_Roll,
	RSS_RemoteCanif_Quad,
	RSS_RemoteCanif_PWM0,
	RSS_RemoteCanif_PWM1,
	RSS_RemoteCanif_PWM2,
	RSS_RemoteCanif_PWM3,
} RemoteSensorSource_t;

typedef enum _SensorTermOrdinal_t {
	SensorTermOrdinal_Sum0 = 0,
	SensorTermOrdinal_Sum1 = 0,
	SensorTermOrdinal_Diff0 = 0,
	SensorTermOrdinal_Diff1 = 0,
}SensorTermOrdinal_t;

#endif // signalTypes__h_
