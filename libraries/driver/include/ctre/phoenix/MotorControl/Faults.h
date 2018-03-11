#pragma once
#include <sstream>
namespace ctre {
namespace phoenix {
namespace motorcontrol {

struct Faults {
	bool UnderVoltage;
	bool ForwardLimitSwitch;
	bool ReverseLimitSwitch;
	bool ForwardSoftLimit;
	bool ReverseSoftLimit;
	bool HardwareFailure;
	bool ResetDuringEn;
	bool SensorOverflow;
	bool SensorOutOfPhase;
	bool HardwareESDReset;
	bool RemoteLossOfSignal;
	//!< True iff any of the above flags are true.
	bool HasAnyFault() const {
		return 	UnderVoltage |
				ForwardLimitSwitch |
				ReverseLimitSwitch |
				ForwardSoftLimit |
				ReverseSoftLimit |
				HardwareFailure |
				ResetDuringEn |
				SensorOverflow |
				SensorOutOfPhase |
				HardwareESDReset |
				RemoteLossOfSignal;
	}
	int ToBitfield() const {
		int retval = 0;
		int mask = 1;
		retval |= UnderVoltage ? mask : 0; mask <<= 1;
		retval |= ForwardLimitSwitch ? mask : 0; mask <<= 1;
		retval |= ReverseLimitSwitch ? mask : 0; mask <<= 1;
		retval |= ForwardSoftLimit ? mask : 0; mask <<= 1;
		retval |= ReverseSoftLimit ? mask : 0; mask <<= 1;
		retval |= HardwareFailure ? mask : 0; mask <<= 1;
		retval |= ResetDuringEn ? mask : 0; mask <<= 1;
		retval |= SensorOverflow ? mask : 0; mask <<= 1;
		retval |= SensorOutOfPhase ? mask : 0; mask <<= 1;
		retval |= HardwareESDReset ? mask : 0; mask <<= 1;
		retval |= RemoteLossOfSignal ? mask : 0; mask <<= 1;
		return retval;
	}
	Faults(int bits) {
		int mask = 1;
		UnderVoltage = (bits & mask) ? true : false; mask <<= 1;
		ForwardLimitSwitch = (bits & mask) ? true : false; mask <<= 1;
		ReverseLimitSwitch = (bits & mask) ? true : false; mask <<= 1;
		ForwardSoftLimit = (bits & mask) ? true : false; mask <<= 1;
		ReverseSoftLimit = (bits & mask) ? true : false; mask <<= 1;
		HardwareFailure = (bits & mask) ? true : false; mask <<= 1;
		ResetDuringEn = (bits & mask) ? true : false; mask <<= 1;
		SensorOverflow = (bits & mask) ? true : false; mask <<= 1;
		SensorOutOfPhase = (bits & mask) ? true : false; mask <<= 1;
		HardwareESDReset = (bits & mask) ? true : false; mask <<= 1;
		RemoteLossOfSignal = (bits & mask) ? true : false; mask <<= 1;
	}
	Faults() {
		UnderVoltage = false;
		ForwardLimitSwitch = false;
		ReverseLimitSwitch = false;
		ForwardSoftLimit = false;
		ReverseSoftLimit = false;
		HardwareFailure =false;
		ResetDuringEn = false;
		SensorOverflow = false;
		SensorOutOfPhase = false;
		HardwareESDReset = false;
		RemoteLossOfSignal = false;
	}
	std::string ToString() {
		std::stringstream work;
		work << " UnderVoltage:" << (UnderVoltage ? "1" : "0");
		work << " ForwardLimitSwitch:" << (ForwardLimitSwitch ? "1" : "0");
		work << " ReverseLimitSwitch:" << (ReverseLimitSwitch ? "1" : "0");
		work << " ForwardSoftLimit:" << (ForwardSoftLimit ? "1" : "0");
		work << " ReverseSoftLimit:" << (ReverseSoftLimit ? "1" : "0");
		work << " HardwareFailure:" << (HardwareFailure ? "1" : "0");
		work << " ResetDuringEn:" << (ResetDuringEn ? "1" : "0");
		work << " SensorOverflow:" << (SensorOverflow ? "1" : "0");
		work << " SensorOutOfPhase:" << (SensorOutOfPhase ? "1" : "0");
		work << " HardwareESDReset:" << (HardwareESDReset ? "1" : "0");
		work << " RemoteLossOfSignal:" << (RemoteLossOfSignal ? "1" : "0");
		return work.str();
	}
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
