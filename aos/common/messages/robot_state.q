package aos;

struct Joystick {
  // A bitmask of the button state.
  uint16_t buttons;

  // The 4 joystick axes.
  double[4] axis;
};

message RobotState {
  Joystick[4] joysticks;

  bool test_mode;
  bool fms_attached;
  bool enabled;
  bool autonomous;
  uint16_t team_id;
  // If this is true, then this message isn't actually from the control
  // system and so should not be trusted as evidence that the button inputs
  // etc are actually real and should be acted on.
  // However, most things should ignore this so that sending fake messages is
  // useful for testing.
  bool fake;
};

// The robot_state Queue is checked by all control loops to make sure that the
// joystick code hasn't died.
// It also provides information about whether or not the robot is in autonomous
// mode and what the team_id is.

queue RobotState robot_state;
