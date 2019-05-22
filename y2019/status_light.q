package y2019;

// Published on ".y2019.status_light"
message StatusLight {
  // How bright to make each one. 0 is off, 1 is full on.
  float red;
  float green;
  float blue;
};

// Published on ".y2019.camera_log"
message CameraLog {
  bool log;
};
