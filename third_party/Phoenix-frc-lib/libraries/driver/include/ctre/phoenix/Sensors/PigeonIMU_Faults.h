#pragma once

namespace ctre {
namespace phoenix {
namespace sensors {

struct PigeonIMU_Faults {
	//!< True iff any of the above flags are true.
	bool HasAnyFault() const {
		return false;
	}
	int ToBitfield() const {
		int retval = 0;
		return retval;
	}
	PigeonIMU_Faults(int bits) {
		(void)bits;
	}
	PigeonIMU_Faults() {
	}
};

} // sensors
} // phoenix
} // ctre

