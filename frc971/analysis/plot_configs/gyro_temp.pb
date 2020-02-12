channel {
  name: "/drivetrain"
  type: "frc971.IMUValues"
  alias: "IMU"
}

figure {
  axes {
    line {
      y_signal {
        channel: "IMU"
        field: "temperature"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    ylabel: "C"
  }
}
