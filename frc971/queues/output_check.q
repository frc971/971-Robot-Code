package frc971;

message OutputCheck {
	uint8_t sent_value;
};
// Each message here represents a value that was sent to the cRIO.
// The sent timestamp of the message is when the value was sent.
queue OutputCheck output_check_queue;
