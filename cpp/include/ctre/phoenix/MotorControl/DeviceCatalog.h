#pragma once

#include "IMotorController.h"
#include <vector>

namespace ctre {
namespace phoenix {
namespace motorcontrol {

class DeviceCatalog {
public:
	void Register(IMotorController *motorController) {
		_mcs.push_back(motorController);
	}

	int MotorControllerCount() {
		return _mcs.size();
	}

	IMotorController* Get(int idx) {
		return _mcs[idx];
	}

	DeviceCatalog & GetInstance() {
		if (!_instance)
			_instance = new DeviceCatalog();
		return *_instance;
	}
private:
	std::vector<IMotorController*> _mcs;

	static DeviceCatalog * _instance;
};

}
} // namespace phoenix
}

