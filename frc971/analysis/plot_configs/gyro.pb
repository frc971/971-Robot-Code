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
        field: "gyro_x"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "IMU"
        field: "gyro_y"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "IMU"
        field: "gyro_z"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    ylabel: "rad / sec"
  }
  axes {
    line {
      y_signal {
        channel: "CalcIMU"
        field: "total_acceleration"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "IMU"
        field: "accelerometer_x"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "IMU"
        field: "accelerometer_y"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "IMU"
        field: "accelerometer_z"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    ylabel: "g"
  }
}

figure {
  axes {
    line {
      y_signal {
        channel: "CalcIMU"
        field: "accel_x_rolling_mean"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "CalcIMU"
        field: "accel_y_rolling_mean"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "CalcIMU"
        field: "accel_z_rolling_mean"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    ylabel: "g"
  }
  axes {
    line {
      y_signal {
        channel: "CalcIMU"
        field: "accel_x_rolling_std"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "CalcIMU"
        field: "accel_y_rolling_std"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "CalcIMU"
        field: "accel_z_rolling_std"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    ylabel: "g"
  }
}

figure {
  axes {
    line {
      y_signal {
        channel: "CalcIMU"
        field: "gyro_x_rolling_mean"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "CalcIMU"
        field: "gyro_y_rolling_mean"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "CalcIMU"
        field: "gyro_z_rolling_mean"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    ylabel: "rad / sec"
  }
  axes {
    line {
      y_signal {
        channel: "CalcIMU"
        field: "gyro_x_rolling_std"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "CalcIMU"
        field: "gyro_y_rolling_std"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    line {
      y_signal {
        channel: "CalcIMU"
        field: "gyro_z_rolling_std"
      }
      x_signal {
        channel: "CalcIMU"
        field: "monotonic_timestamp_sec"
      }
    }
    ylabel: "rad / sec"
  }
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
