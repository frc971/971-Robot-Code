package frc971.sensors;

message OtherSensors {
	double sonar_distance;
	double plunger_hall_effect_distance;
};
queue OtherSensors other_sensors;

message AutoMode {
	double voltage;
};
queue AutoMode auto_mode;
