package y2016_bot3.sensors;

message BallDetector {
  // Voltage measured by the ball detector sensor.

  // Higher voltage means ball is closer to detector, lower voltage means ball
  // is far from the sensor or not in the robot at all.
  // TODO(comran): Check to see if our sensor's output corresponds with the
  // comment above.

  double voltage;
};
queue BallDetector ball_detector;
