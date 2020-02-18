channel {
  name: "/drivetrain"
  type: "frc971.control_loops.drivetrain.Status"
  alias: "DrivetrainStatus"
}

figure {
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.yaw"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.lateral_pitch"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.longitudinal_pitch"
      }
    }
    ylabel: "rad"
  }
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.quaternion_x"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.quaternion_y"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.quaternion_z"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.quaternion_w"
      }
    }
  }
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.accel_x"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.accel_y"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.accel_z"
      }
    }
    ylabel: "m/s/s"
  }
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.velocity_x"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.velocity_y"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.velocity_z"
      }
    }
  }
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.position_x"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.position_y"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.position_z"
      }
    }
  }
}

figure {
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "zeroing.gyro_x_average"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "zeroing.gyro_y_average"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "zeroing.gyro_z_average"
      }
    }
  }
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.expected_accel_x"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.expected_accel_y"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.expected_accel_z"
      }
    }
  }
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "down_estimator.gravity_magnitude"
      }
    }
  }
  axes {
    line {
      y_signal {
        channel: "CalcDrivetrainStatus"
        field: "down_estimator.accel_x_rolling_mean"
      }
    }
    line {
      y_signal {
        channel: "CalcDrivetrainStatus"
        field: "down_estimator.accel_y_rolling_mean"
      }
    }
    line {
      y_signal {
        channel: "CalcDrivetrainStatus"
        field: "down_estimator.accel_z_rolling_mean"
      }
    }
  }
}
