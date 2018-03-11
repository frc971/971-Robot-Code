#pragma once

namespace ctre {
namespace phoenix {

/** Enumerated type for status frame types. */
enum CANifierControlFrame {
	CANifier_Control_1_General = 0x03040000,
	CANifier_Control_2_PwmOutput = 0x03040040,
};

} // namespace phoenix
} // namespace ctre
