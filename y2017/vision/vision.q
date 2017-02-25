package y2017.vision;

message VisionStatus {
  bool image_valid;

  // Distance to the target in meters.
  double distance;
  // The angle in radians of the bottom of the target.
  double angle;

  // Capture time of the angle using the clock behind monotonic_clock::now().
  int64_t target_time;
};
queue VisionStatus vision_status;
