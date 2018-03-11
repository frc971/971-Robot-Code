#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

enum NeutralMode {
	/** Use the NeutralMode that is set by the jumper wire on the CAN device */
	EEPROMSetting = 0,
	/** Stop the motor's rotation by applying a force. */
    Coast = 1,
    /** Stop the motor's rotation by applying a force. */
    Brake = 2,
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
