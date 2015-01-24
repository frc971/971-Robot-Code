package frc971;

// Represents all of the data for a single potentiometer and indexed encoder
// pair.
// The units on all of the positions are the same.
// All encoder values are relative to where the encoder was at some arbitrary
// point in time. All potentiometer values are relative to some arbitrary 0
// position which varies with each robot.
struct PotAndIndexPosition {
  // Current position read from the encoder.
  double encoder;
  // Current position read from the potentiometer.
  double pot;

  // Position from the encoder latched at the last index pulse.
  double latched_encoder;
  // Position from the potentiometer latched at the last index pulse.
  double latched_pot;

  // How many index pulses we've seen since startup. Starts at 0.
  uint32_t index_pulses;
};

// A left/right pair of PotAndIndexPositions.
struct PotAndIndexPair {
  PotAndIndexPosition left;
  PotAndIndexPosition right;
};
