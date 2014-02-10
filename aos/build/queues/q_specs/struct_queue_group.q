package aos.test;

struct Claw {
	int32_t a;
	int32_t b;
};

	message HPosition {
		Claw top;
	};

// All angles here are 0 horizontal, positive up. 
queue_group ClawGroup {

	message Position {
		Claw top;
	};
	queue Position kjks;
};

