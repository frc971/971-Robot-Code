package y2019.control_loops.drivetrain;

// See the Target structure in //y2019/jevois:structures.h for documentation.
struct CameraTarget {
  float distance;
  float height;
  float heading;
  float skew;
};

message CameraFrame {
  // Number of nanoseconds since the aos::monotonic_clock epoch at which this
  // frame was captured.
  int64_t timestamp;

  // Number of targets actually in this frame.
  uint8_t num_targets;

  // Buffer for the targets.
  CameraTarget[3] targets;

  // Index of the camera position (not serial number) which this frame is from.
  uint8_t camera;
};

queue CameraFrame camera_frames;
