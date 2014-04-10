package aos.controls;

// Each different set of values here represents a different set of sensor
// values (ie different encoder zeros etc).
message SensorGeneration {
	// The PID of the process reading sensor values.
	int32_t reader_pid;
	// A count of how many times a sensor reading process sees the cape reset.
	uint32_t cape_resets;
};

// A new message is placed on here by the process that reads sensor values each
// time it starts up or detects the cape resetting.
queue SensorGeneration sensor_generation;
