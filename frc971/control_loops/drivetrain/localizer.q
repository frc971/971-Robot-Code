package frc971.control_loops.drivetrain;

// Allows you to reset the state of the localizer to a specific position on the
// field.
message LocalizerControl {
  float x;      // X position, meters
  float y;      // Y position, meters
  float theta;  // heading, radians
  double theta_uncertainty; // Uncertainty in theta.
};

queue LocalizerControl localizer_control;
