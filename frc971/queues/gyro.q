package frc971.sensors;

// Published on ".frc971.sensors.gyro_reading"
message GyroReading {
	// Positive is counter-clockwise (Austin says "it's Positive").
	// Right-hand coordinate system around the Z-axis going up.
  // The angle is measured in radians.
	double angle;
  // The angular velocity in radians/sec
	double velocity;
};

// Published on ".frc971.sensors.gyro_part_id"
message Uid {
	uint32_t uid;
};
