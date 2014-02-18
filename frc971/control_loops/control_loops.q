package frc971;

// Records edges captured on a single hall effect sensor.
struct HallEffectStruct {
  bool current;
  int32_t posedge_count;
  int32_t negedge_count;
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
