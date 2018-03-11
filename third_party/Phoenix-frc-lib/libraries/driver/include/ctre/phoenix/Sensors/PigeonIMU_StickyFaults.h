#pragma once

namespace ctre {
namespace phoenix {
namespace sensors {

struct PigeonIMU_StickyFaults {
	//!< True iff any of the above flags are true.
	bool HasAnyFault() const {
		return false;
	}
	int ToBitfield() const {
		int retval = 0;
		return retval;
	}
	PigeonIMU_StickyFaults(int bits) {
		(void)bits;
	}
	PigeonIMU_StickyFaults() {
	}
};

} // sensors
} // phoenix
} // ctre
