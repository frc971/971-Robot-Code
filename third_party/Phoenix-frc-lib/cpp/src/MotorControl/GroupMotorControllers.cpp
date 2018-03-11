#include "ctre/phoenix/MotorControl/GroupMotorControllers.h"

using namespace ctre::phoenix;

namespace ctre {
namespace phoenix  {
namespace motorcontrol {

std::vector<IMotorController*> GroupMotorControllers::_mcs;

void GroupMotorControllers::Register(IMotorController *motorController) {
	_mcs.push_back(motorController);
}

int GroupMotorControllers::MotorControllerCount() {
	return _mcs.size();
}

IMotorController* GroupMotorControllers::Get(int idx) {
	return _mcs[idx];
}

}
}
}
