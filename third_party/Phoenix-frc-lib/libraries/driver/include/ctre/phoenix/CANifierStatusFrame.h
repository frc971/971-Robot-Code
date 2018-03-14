#pragma once

namespace ctre {
namespace phoenix {

/** Enumerated type for status frame types. */
enum CANifierStatusFrame {
	CANifierStatusFrame_Status_1_General = 0x041400,
	CANifierStatusFrame_Status_2_General = 0x041440,
	CANifierStatusFrame_Status_3_PwmInputs0 = 0x041480,
	CANifierStatusFrame_Status_4_PwmInputs1 = 0x0414C0,
	CANifierStatusFrame_Status_5_PwmInputs2 = 0x041500,
	CANifierStatusFrame_Status_6_PwmInputs3 = 0x041540,
	CANifierStatusFrame_Status_8_Misc = 0x0415C0,
};

} // namespace phoenix
} // namespace ctre
