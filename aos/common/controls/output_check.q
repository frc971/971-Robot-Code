package aos.controls;

message OutputCheck {
	// The 1-255 sent to the FPGA.
	uint8_t pwm_value;
	// The length of the pulse in milliseconds.
	double pulse_length;
};

// Each message here represents a value that was sent to the cRIO.
// The sent timestamp of the message is when the value was sent.
queue OutputCheck output_check_sent;

// Each message here represents a value that was received by the sensor.
// The sent timestamp of the message is when it was received.
queue OutputCheck output_check_received;
