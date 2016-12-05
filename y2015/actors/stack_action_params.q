package y2015.actors;

// Parameters to send with start.
struct StackParams {
  // If true, don't grab the lower tote after lowering.
  bool only_place;

  // The angle to move the arm to while lowering it.
  double arm_clearance;

  double claw_out_angle;
  // The height just above the box to move before lowering.
  double over_box_before_place_height;

  // Bottom position.
  double bottom;
};
