#pragma once
#include <stdint.h>
/**
 * Simple address holder.  This will be removed TBD.
 */
class CANBusAddressable {

public:
	CANBusAddressable(uint8_t deviceNumber) {
		_deviceNum = deviceNumber;
	}

	uint8_t GetDeviceNumber() {
		return _deviceNum;
	}
protected:
private:
	uint32_t _deviceNum;
};

