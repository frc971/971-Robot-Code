package frc971.autonomous;

message AutoControl {
  // True if auto mode should be running, false otherwise.
  bool run_auto;
};

queue AutoControl autonomous;
