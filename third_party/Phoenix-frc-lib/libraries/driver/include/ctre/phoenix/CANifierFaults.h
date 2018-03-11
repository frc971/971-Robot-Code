#pragma once

namespace ctre {
namespace phoenix {

struct CANifierFaults {
	//!< True iff any of the above flags are true.
	bool HasAnyFault() const {
		return false;
	}
	int ToBitfield() const {
		int retval = 0;
		return retval;
	}
	CANifierFaults(int bits) {
		(void)bits;
	}
	CANifierFaults() {
	}
};

} // phoenix
} // ctre

