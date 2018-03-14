package com.ctre.phoenix;

public class CANifierStickyFaults {
	//!< True iff any of the above flags are true.
	public boolean hasAnyFault() {
		return false;
	}
	public int toBitfield() {
		int retval = 0;
		return retval;
	}
	public void update(int bits) {
	}
	public CANifierStickyFaults() {
	}
};
