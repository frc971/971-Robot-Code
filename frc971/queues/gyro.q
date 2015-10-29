package frc971.sensors;

message GyroReading {
	// Positive is counter-clockwise (Austin says "it's Positive").
	// Right-hand coordinate system around the Z-axis going up.
  // The angle is measured in radians.
	double angle;
  // The angular velocity in radians/sec
	double velocity;
};
queue GyroReading gyro_reading;

message Uid {
	uint32_t uid;
};
queue Uid gyro_part_id;
