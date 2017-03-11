package frc971;

// Represents all of the data for a single indexed encoder. In other words,
// just a relative encoder with an index pulse.
// The units on all of the positions are the same.
// All encoder values are relative to where the encoder was at some arbitrary
// point in time. All potentiometer values are relative to some arbitrary 0
// position which varies with each robot.
struct IndexPosition {
  // Current position read from the encoder.
  double encoder;
  // Position from the encoder latched at the last index pulse.
  double latched_encoder;
  // How many index pulses we've seen since startup. Starts at 0.
  uint32_t index_pulses;
};

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

// Represents all of the data for a single potentiometer with an absolute and
// relative encoder pair.
// The units on all of the positions are the same.
// The relative encoder values are relative to where the encoder was at some
// arbitrary point in time. All potentiometer values are relative to some
// arbitrary 0 position which varies with each robot.
struct PotAndAbsolutePosition {
  // Current position read from each encoder.
  double encoder;
  double absolute_encoder;

  // Current position read from the potentiometer.
  double pot;
};

// The internal state of a zeroing estimator.
struct EstimatorState {
  // If true, there has been a fatal error for the estimator.
  bool error;
  // If the joint has seen an index pulse and is zeroed.
  bool zeroed;
  // The estimated position of the joint.
  double position;

  // The estimated position not using the index pulse.
  double pot_position;
};

// The internal state of a zeroing estimator.
struct AbsoluteEstimatorState {
  // If true, there has been a fatal error for the estimator.
  bool error;
  // If the joint has seen an index pulse and is zeroed.
  bool zeroed;
  // The estimated position of the joint.
  double position;

  // The estimated position not using the index pulse.
  double pot_position;

  // The estimated absolute position of the encoder.  This is filtered, so it
  // can be easily used when zeroing.
  double absolute_position;
};

// The internal state of a zeroing estimator.
struct IndexEstimatorState {
  // If true, there has been a fatal error for the estimator.
  bool error;
  // If the joint has seen an index pulse and is zeroed.
  bool zeroed;
  // The estimated position of the joint. This is just the position relative to
  // where we started if we're not zeroed yet.
  double position;

  // The positions of the extreme index pulses we've seen.
  double min_index_position;
  double max_index_position;
  // The number of index pulses we've seen.
  int32_t index_pulses_seen;
};

struct HallEffectAndPositionEstimatorState {
  // If error.
  bool error;
  // If we've found a positive edge while moving backwards and is zeroed.
  bool zeroed;
  // Encoder angle relative to where we started.
  double encoder;
  // The positions of the extreme posedges we've seen.
  // If we've gotten enough samples where the hall effect is high before can be
  // certain it is not a false positive.
  bool high_long_enough;
  double offset;
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

// Records the hall effect sensor and encoder values.
struct HallEffectAndPosition {
  // The current hall effect state.
  bool current;
  // The current encoder position.
  double position;
  // The number of positive and negative edges we've seen on the hall effect
  // sensor.
  int32_t posedge_count;
  int32_t negedge_count;
  // The values corresponding to the last hall effect sensor reading.
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

// Parameters for the motion profiles.
struct ProfileParameters {
  // Maximum velocity for the profile.
  float max_velocity;
  // Maximum acceleration for the profile.
  float max_acceleration;
};
