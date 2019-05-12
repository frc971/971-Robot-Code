package y2019.control_loops.drivetrain;

// A message to provide information to the target selector about what it should
// The drivetrain listens on ".y2019.control_loops.drivetrain.target_selector_hint"
message TargetSelectorHint {
  // Which target we should go for:
  // 0 implies no selection, we should just default to whatever.
  // 1, 2, and 3 imply the near, middle, and far targets.
  // 4 implies far side of the rocket ship.
  // These should match the SelectionHint enum in target_selector.h.
  uint8_t suggested_target;
};
