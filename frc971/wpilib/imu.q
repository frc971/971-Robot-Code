package frc971;

// Values returned from an IMU.
message IMUValues {
  // Gyro readings in radians/second.
  // Positive is clockwise looking at the connector.
  float gyro_x;
  // Positive is clockwise looking at the right side (from the connector).
  float gyro_y;
  // Positive is counterclockwise looking at the top.
  float gyro_z;

  // Accelerometer readings in Gs.
  // Positive is up.
  float accelerometer_x;
  // Positive is away from the right side (from the connector).
  float accelerometer_y;
  // Positive is away from the connector.
  float accelerometer_z;

  // Magnetometer readings in gauss.
  // Positive is up.
  float magnetometer_x;
  // Positive is away from the right side (from the connector).
  float magnetometer_y;
  // Positive is away from the connector.
  float magnetometer_z;

  // Barometer readings in pascals.
  float barometer;

  // Temperature readings in degrees Celsius.
  float temperature;

  // FPGA timestamp when the values were captured.
  double fpga_timestamp;
  // CLOCK_MONOTONIC time in nanoseconds when the values were captured.
  int64_t monotonic_timestamp_ns;
};

queue IMUValues imu_values;
