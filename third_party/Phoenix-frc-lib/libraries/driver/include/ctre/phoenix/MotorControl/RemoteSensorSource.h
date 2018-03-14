#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

enum class RemoteSensorSource {
	RemoteSensorSource_Off,
	RemoteSensorSource_TalonSRX_SelectedSensor,
	RemoteSensorSource_Pigeon_Yaw,
	RemoteSensorSource_Pigeon_Pitch,
	RemoteSensorSource_Pigeon_Roll,
	RemoteSensorSource_CANifier_Quadrature,
	RemoteSensorSource_CANifier_PWMInput0,
	RemoteSensorSource_CANifier_PWMInput1,
	RemoteSensorSource_CANifier_PWMInput2,
	RemoteSensorSource_CANifier_PWMInput3,
	RemoteSensorSource_GadgeteerPigeon_Yaw,
	RemoteSensorSource_GadgeteerPigeon_Pitch,
	RemoteSensorSource_GadgeteerPigeon_Roll,
};

}
}
}
