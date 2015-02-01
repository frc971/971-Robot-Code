package frc971.zeroing;

struct ZeroingInfo {
	double pot;
	double encoder;
	double index_encoder;
	int32_t index_count;
};

message TestMessage {
	int32_t test_int;
};

queue TestMessage test_queue;
