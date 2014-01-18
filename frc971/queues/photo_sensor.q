package frc971.sensors;

message BlockedSensor {
	bool blocked;
};

queue BlockedSensor bottom_ball_sensor;
queue BlockedSensor top_ball_sensor;
queue BlockedSensor left_bump;
queue BlockedSensor right_bump;
