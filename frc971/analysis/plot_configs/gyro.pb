channel {
  name: "/drivetrain"
  type: "frc971.IMUValues"
  alias: "IMU"
}

figure {
  axes {
    signal {
      channel: "IMU"
      field: "gyro_x"
    }
    signal {
      channel: "IMU"
      field: "gyro_y"
    }
    signal {
      channel: "IMU"
      field: "gyro_z"
    }
    ylabel: "rad / sec"
  }
  axes {
    signal {
      channel: "IMU"
      field: "accelerometer_x"
    }
    signal {
      channel: "IMU"
      field: "accelerometer_y"
    }
    signal {
      channel: "IMU"
      field: "accelerometer_z"
    }
    ylabel: "g"
  }
}
