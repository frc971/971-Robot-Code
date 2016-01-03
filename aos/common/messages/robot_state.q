package aos;

struct Joystick {
  // A bitmask of the button state.
  uint16_t buttons;

  // The 4 joystick axes.
  double[4] axis;

  // The POV axis.
  int32_t pov;
};

message JoystickState {
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
  // useful for testing. The only difference in behavior should be motors not
  // actually turning on.
  bool fake;
};

// This queue is checked by all control loops to make sure that the
// joystick code hasn't died.
queue JoystickState joystick_state;

message RobotState {
  // The PID of the process reading sensors.
  // This is here so control loops can tell when it changes.
  int32_t reader_pid;

  // True when outputs are enabled.
  // Motor controllers keep going for a bit after this goes to false.
  bool outputs_enabled;
  // Indicates whether something is browned out (I think motor controller
  // outputs). IMPORTANT: This is NOT !outputs_enabled. outputs_enabled goes to
  // false for other reasons too (disabled, e-stopped, maybe more).
  bool browned_out;

  // Whether the two sensor rails are currently working.
  bool is_3v3_active;
  bool is_5v_active;
  // The current voltages measured on the two sensor rails.
  double voltage_3v3;
  double voltage_5v;

  // The input voltage to the roboRIO.
  double voltage_roborio_in;

  // From the DriverStation object, aka what FMS sees and what shows up on the
  // actual driver's station.
  double voltage_battery;
};

// Messages are sent out on this queue along with reading sensors. It contains
// global robot state and information about whether the process reading sensors
// has been restarted, along with all counters etc it keeps track of.
queue RobotState robot_state;
