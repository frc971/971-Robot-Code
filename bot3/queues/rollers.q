package bot3;

message Rollers {
  // Positive voltage = intaking, Negative = spitting.
  double front_intake_voltage;
  double back_intake_voltage;
  // Voltage for the low goal rollers.
  // Positive voltage = ball towards back, Negative = ball towards front.
  double low_goal_voltage;

  // Whether the front and back intake pistons are extended.
  bool front_extended;
  bool back_extended;
};
queue Rollers rollers;
