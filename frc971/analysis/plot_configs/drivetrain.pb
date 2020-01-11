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
    signal {
      channel: "JoystickState"
      field: "test_mode"
    }
    signal {
      channel: "JoystickState"
      field: "autonomous"
    }
    signal {
      channel: "RobotState"
      field: "browned_out"
    }
    signal {
      channel: "JoystickState"
      field: "enabled"
    }
    ylabel: "[bool]"
  }
  axes {
    signal {
      channel: "RobotState"
      field: "voltage_battery"
    }
    ylabel: "[V]"
  }
  axes {
    signal {
      channel: "Status"
      field: "line_follow_logging.frozen"
    }
    signal {
      channel: "Goal"
      field: "quickturn"
    }
    ylabel: "[bool]"
  }
}

figure {
  axes {
    signal {
      channel: "Status"
      field: "poly_drive_logging.ff_left_voltage"
    }
    signal {
      channel: "Status"
      field: "poly_drive_logging.ff_right_voltage"
    }
    signal {
      channel: "Output"
      field: "left_voltage"
    }
    signal {
      channel: "Output"
      field: "right_voltage"
    }
    ylabel: "[V]"
  }
  axes {
    signal {
      channel: "Status"
      field: "theta"
    }
    ylabel: "[rad]"
  }
  axes {
    signal {
      channel: "Status"
      field: "robot_speed"
    }
    signal {
      channel: "Status"
      field: "trajectory_logging.left_velocity"
    }
    signal {
      channel: "Status"
      field: "poly_drive_logging.goal_left_velocity"
    }
    signal {
      channel: "Status"
      field: "estimated_left_velocity"
    }
    signal {
      channel: "Status"
      field: "trajectory_logging.right_velocity"
    }
    signal {
      channel: "Status"
      field: "poly_drive_logging.goal_right_velocity"
    }
    signal {
      channel: "Status"
      field: "estimated_right_velocity"
    }
    ylabel: "[m/s]"
  }
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
