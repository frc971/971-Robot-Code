package frc971.autonomous;

message CanGrabberControl {
  // Voltage to send out to can grabbers.
  double can_voltage;
};
queue CanGrabberControl can_control;
