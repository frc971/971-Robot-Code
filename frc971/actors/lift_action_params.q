package frc971.actors;

// Parameters to send with start.
struct LiftParams {
  // Lift height
  double lift_height;
  // Arm goal.
  double lift_arm;

  // If true, do the second lift.
  bool second_lift;
  // Arm goal.
  double intermediate_lift_height;

  // True to move the claw in the middle of the lift.
  bool pack_claw;
  // Iff pack_claw is true, the angle to move the claw to.
  double pack_claw_angle;
};
