channel {
  name: "/aos"
  type: "aos.JoystickState"
  alias: "JoystickState"
}
channel {
  name: "/aos"
  type: "aos.RobotState"
  alias: "RobotState"
}
channel {
  name: "/drivetrain"
  type: "frc971.control_loops.drivetrain.Goal"
  alias: "Goal"
}
channel {
  name: "/drivetrain"
  type: "frc971.control_loops.drivetrain.Status"
  alias: "Status"
}
channel {
  name: "/drivetrain"
  type: "frc971.control_loops.drivetrain.Output"
  alias: "Output"
}
channel {
  name: "/drivetrain"
  type: "frc971.IMUValues"
  alias: "IMU"
}

figure {
  axes {
    line {
      y_signal {
        channel: "JoystickState"
        field: "test_mode"
      }
    }
    line {
      y_signal {
        channel: "JoystickState"
        field: "autonomous"
      }
    }
    line {
      y_signal {
        channel: "RobotState"
        field: "browned_out"
      }
    }
    line {
      y_signal {
        channel: "JoystickState"
        field: "enabled"
      }
    }
    ylabel: "[bool]"
  }
  axes {
    line {
      y_signal {
        channel: "RobotState"
        field: "voltage_battery"
      }
    }
    ylabel: "[V]"
  }
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "line_follow_logging.frozen"
      }
    }
    line {
      y_signal {
        channel: "Goal"
        field: "quickturn"
      }
    }
    ylabel: "[bool]"
  }
}

figure {
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "poly_drive_logging.ff_left_voltage"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "poly_drive_logging.ff_right_voltage"
      }
    }
    line {
      y_signal {
        channel: "Output"
        field: "left_voltage"
      }
    }
    line {
      y_signal {
        channel: "Output"
        field: "right_voltage"
      }
    }
    ylabel: "[V]"
  }
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "theta"
      }
    }
    ylabel: "[rad]"
  }
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "robot_speed"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "trajectory_logging.left_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "poly_drive_logging.goal_left_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "estimated_left_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "trajectory_logging.right_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "poly_drive_logging.goal_right_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "estimated_right_velocity"
      }
    }
    ylabel: "[m/s]"
  }
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
        channel: "Status"
        field: "down_estimator.yaw"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "down_estimator.lateral_pitch"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "down_estimator.longitudinal_pitch"
      }
    }
    ylabel: "rad"
  }
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "down_estimator.quaternion_x"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "down_estimator.quaternion_y"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "down_estimator.quaternion_z"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "down_estimator.quaternion_w"
      }
    }
  }
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "down_estimator.velocity_x"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "down_estimator.velocity_y"
      }
    }
  }
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "down_estimator.position_x"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "down_estimator.position_y"
      }
    }
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
