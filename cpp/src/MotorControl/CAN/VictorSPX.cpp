#include "ctre/phoenix/MotorControl/CAN/VictorSPX.h"
#include "HAL/HAL.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::motorcontrol::can;
/**
 * Constructor
 * @param deviceNumber [0,62]
 */
VictorSPX::VictorSPX(int deviceNumber) :
    BaseMotorController(deviceNumber | 0x01040000) {
		HAL_Report(HALUsageReporting::kResourceType_CTRE_future1, deviceNumber + 1);
	}
