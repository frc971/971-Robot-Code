package frc971.control_loops.drivetrain;

// Allows you to reset the state of the localizer to a specific position on the
// field.
// Published on ".frc971.control_loops.drivetrain.localizer_control"
message LocalizerControl {
  float x;      // X position, meters
  float y;      // Y position, meters
  float theta;  // heading, radians
  double theta_uncertainty; // Uncertainty in theta.
  bool keep_current_theta; // Whether to keep the current theta value.
};
