#pragma once

namespace ctre {
namespace phoenix {

struct CANifierStickyFaults {
	//!< True iff any of the above flags are true.
	bool HasAnyFault() const {
		return false;
	}
	int ToBitfield() const {
		int retval = 0;
		return retval;
	}
	CANifierStickyFaults(int bits) {
		(void)bits;
	}
	CANifierStickyFaults() {
	}
};

} // phoenix
} // ctre
