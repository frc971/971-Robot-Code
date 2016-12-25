package y2016.vision;

message VisionStatus {
  bool left_image_valid;
  bool right_image_valid;
  // Times when the images were taken as nanoseconds on CLOCK_MONOTONIC on the
  // TK1.
  int64_t left_image_timestamp;
  int64_t right_image_timestamp;
  // Times when the images were sent from the TK1 as nanoseconds on the TK1's
  // CLOCK_MONOTONIC.
  int64_t left_send_timestamp;
  int64_t right_send_timestamp;

  // Horizontal angle of the goal in radians.
  // TODO(Brian): Figure out which way is positive.
  double horizontal_angle;
  // Vertical angle of the goal in radians.
  // TODO(Brian): Figure out which way is positive.
  double vertical_angle;
  // Distance to the target in meters.
  double distance;
  // The angle in radians of the bottom of the target.
  double angle;

  // Capture time of the angle using the clock behind monotonic_clock::now().
  int64_t target_time;

  // The estimated positions of both sides of the drivetrain when the frame
  // was captured.
  // These are the estimated_left_position and estimated_right_position members
  // of the drivetrain queue.
  double drivetrain_left_position;
  double drivetrain_right_position;
};
queue VisionStatus vision_status;
