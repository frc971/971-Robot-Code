package aos;

message RobotState {
	bool enabled;
	bool autonomous;
	uint16_t team_id;
};

// The robot_state Queue is checked by all control loops to make sure that the
// joystick code hasn't died.
// It also provides information about whether or not the robot is in autonomous
// mode and what the team_id is.

queue RobotState robot_state;
