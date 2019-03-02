package y2019.control_loops.drivetrain;

// These structures have a nearly one-to-one correspondence to those in
// //y2019/jevois:structures.h. Please refer to that file for details.
struct CameraTarget {
  float distance;
  float height;
  float heading;
  float skew;
};

message CameraFrame {
  // monotonic time in nanoseconds at which frame was taken (note structure.h
  // uses age).
  int64_t timestamp;

  // Number of targets actually in this frame.
  uint8_t num_targets;

  // Buffer for the targets
  CameraTarget[3] targets;

  // Index of the camera with which this frame was taken:
  uint8_t camera;
};

queue CameraFrame camera_frames;
