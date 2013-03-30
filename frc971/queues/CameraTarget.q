package frc971.vision;

message CameraTarget {
  double percent_azimuth_off_center;
  double percent_elevation_off_center;
  // todo:(pschuh) add time syntax when havith more sleep
  uint64_t timestamp; 
};

message TargetAngle {
  double target_angle;
};

queue TargetAngle target_angle;
queue CameraTarget targets;
