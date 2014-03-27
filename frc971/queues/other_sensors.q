package frc971.sensors;

message OtherSensors {
	double sonar_distance;
	double plunger_hall_effect_distance;
};
queue OtherSensors other_sensors;

message GyroReading {
	double angle;
};
queue GyroReading gyro_reading;
