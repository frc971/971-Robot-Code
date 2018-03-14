#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

enum StatusFrameEnhanced {
	Status_1_General = 0x1400,
	Status_2_Feedback0 = 0x1440,
	Status_4_AinTempVbat = 0x14C0,
	Status_6_Misc = 0x1540,
	Status_7_CommStatus = 0x1580,
	Status_9_MotProfBuffer = 0x1600,
	/**
	 * Old name for Status 10 Frame.
	 * Use Status_10_Targets instead.
	 */
	Status_10_MotionMagic = 0x1640,
	/**
	 * Correct name for Status 10 Frame.
	 * Functionally equivalent to Status_10_MotionMagic
	 */
	Status_10_Targets = 0x1640,
	Status_12_Feedback1 = 0x16C0,
	Status_13_Base_PIDF0 = 0x1700,
	Status_14_Turn_PIDF1 = 0x1740,
	Status_15_FirmareApiStatus = 0x1780,

	Status_3_Quadrature = 0x1480,
	Status_8_PulseWidth = 0x15C0,
	Status_11_UartGadgeteer = 0x1680,
};

enum StatusFrame {
	Status_1_General_ = 0x1400,
	Status_2_Feedback0_ = 0x1440,
	Status_4_AinTempVbat_ = 0x14C0,
	Status_6_Misc_ = 0x1540,
	Status_7_CommStatus_ = 0x1580,
	Status_9_MotProfBuffer_ = 0x1600,
	/**
	 * Old name for Status 10 Frame.
	 * Use Status_10_Targets instead.
	 */
	Status_10_MotionMagic_ = 0x1640,
	/**
	 * Correct name for Status 10 Frame.
	 * Functionally equivalent to Status_10_MotionMagic
	 */
	Status_10_Targets_ = 0x1640,
	Status_12_Feedback1_ = 0x16C0,
	Status_13_Base_PIDF0_ = 0x1700,
	Status_14_Turn_PIDF1_ = 0x1740,
	Status_15_FirmareApiStatus_ = 0x1780,
};
class StatusFrameRoutines {
public:
	StatusFrameEnhanced Promote(StatusFrame statusFrame) {
		return (StatusFrameEnhanced) statusFrame;
	}
};
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
