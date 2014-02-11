package aos.test;

import "q_specs/struct_test.q";


message Position {
	int32_t a;
	int32_t[2] b;
	.aos.test2.ClawHalfPosition top;
	.aos.test2.ClawHalfPosition bottom;
	bool c;
};
