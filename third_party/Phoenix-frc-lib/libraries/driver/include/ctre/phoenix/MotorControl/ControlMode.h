#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

enum class ControlMode {
	PercentOutput = 0,
	Position = 1,
	Velocity = 2,
	Current = 3,
	Follower = 5,
	MotionProfile = 6,
	MotionMagic = 7,
	MotionProfileArc = 10,

	Disabled = 15,
};

}
}
}
