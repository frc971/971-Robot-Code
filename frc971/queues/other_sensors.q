package frc971.sensors;

message OtherSensors {
	double sonar_distance;
	double plunger_hall_effect_distance;
};
queue OtherSensors other_sensors;

message GyroReading {
	// Positive is counter-clockwise (Austin says "it's Positive").
	// Right-hand coordinate system around the Z-axis going up.
	double angle;
};
queue GyroReading gyro_reading;
