package y2019;

message StatusLight {
  // How bright to make each one. 0 is off, 1 is full on.
  float red;
  float green;
  float blue;
};

queue StatusLight status_light;

message CameraLog {
  bool log;
};

queue CameraLog camera_log;
