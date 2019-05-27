package y2016.sensors;

// Published on ".y2016.sensors.ball_detector"
message BallDetector {
  // Voltage measured by the ball detector sensor.

  // Higher voltage means ball is closer to detector, lower voltage means ball
  // is far from the sensor or not in the robot at all.
  // TODO(comran): Check to see if our sensor's output corresponds with the
  // comment above.

  double voltage;
};
