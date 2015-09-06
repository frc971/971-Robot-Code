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

// The internal state of a zeroing estimator.
struct EstimatorState {
  // If true, there has been a fatal error for the estimator.
  bool error;
  // If the joint has seen an index pulse and is zeroed.
  bool zeroed;
  // The estimated position of the joint.
  double position;
};

// A left/right pair of PotAndIndexPositions.
struct PotAndIndexPair {
  PotAndIndexPosition left;
  PotAndIndexPosition right;
};

// Records edges captured on a single hall effect sensor.
struct HallEffectStruct {
  bool current;
  int32_t posedge_count;
  int32_t negedge_count;
  double posedge_value;
  double negedge_value;
};

// Records the positions for a mechanism with edge-capturing sensors on it.
struct HallEffectPositions {
  double current;
  double posedge;
  double negedge;
};

// Records edges captured on a single hall effect sensor.
struct PosedgeOnlyCountedHallEffectStruct {
  bool current;
  int32_t posedge_count;
  int32_t negedge_count;
  double posedge_value;
};
