#pragma once

#include "IMotorController.h"
#include <vector>

namespace ctre {
namespace phoenix {
namespace motorcontrol {

class GroupMotorControllers {
public:
	static void Register(IMotorController *motorController);
	static int MotorControllerCount();
	static IMotorController* Get(int idx);

private:
	static std::vector<IMotorController*> _mcs;
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre

