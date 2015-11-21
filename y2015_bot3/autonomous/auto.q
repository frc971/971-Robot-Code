package y2015_bot3.autonomous;

message AutoControl {
  // True if auto mode should be running, false otherwise.
  bool run_auto;
};
queue AutoControl autonomous;

message CanGrabberControl {
  // Voltage to send out to can grabbers.
  double can_grabber_voltage;
  // Can grabbers fired
  bool can_grabbers;
};
queue CanGrabberControl can_grabber_control;
