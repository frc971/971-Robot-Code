#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/* forward proto */
class IMotorController;

class IFollower {
public:
	virtual ~IFollower(){}
	virtual void Follow(ctre::phoenix::motorcontrol::IMotorController & masterToFollow) = 0;
	virtual void ValueUpdated()= 0;
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
