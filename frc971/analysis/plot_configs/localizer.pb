channel {
  name: "/drivetrain"
  type: "frc971.control_loops.drivetrain.Status"
  alias: "DrivetrainStatus"
}
channel {
  name: "/drivetrain/truth"
  type: "frc971.control_loops.drivetrain.Status"
  alias: "DrivetrainTruthStatus"
}
channel {
  name: "/drivetrain"
  type: "frc971.control_loops.drivetrain.Position"
  alias: "DrivetrainPosition"
}
channel {
  name: "/drivetrain"
  type: "frc971.control_loops.drivetrain.Output"
  alias: "DrivetrainOutput"
}

figure {
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "trajectory_logging.y"
      }
      x_signal {
        channel: "DrivetrainStatus"
        field: "trajectory_logging.x"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainTruthStatus"
        field: "y"
      }
      x_signal {
        channel: "DrivetrainTruthStatus"
        field: "x"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "y"
      }
      x_signal {
        channel: "DrivetrainStatus"
        field: "x"
      }
    }
    share_x_axis: false
    xlabel: "x (m)"
    ylabel: "y (m)"
  }
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
        channel: "DrivetrainTruthStatus"
        field: "theta"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.theta"
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
        channel: "DrivetrainTruthStatus"
        field: "x"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainTruthStatus"
        field: "y"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.x"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.y"
      }
    }
    ylabel: "m"
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
        field: "localizer.longitudinal_velocity_offset"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.lateral_velocity"
      }
    }
    ylabel: "m/s"
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
}

figure {
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.left_velocity"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.right_velocity"
      }
    }
    ylabel: "m/s"
  }
  axes {
    line {
      y_signal {
        channel: "DrivetrainPosition"
        field: "left_encoder"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainPosition"
        field: "right_encoder"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.left_encoder"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.right_encoder"
      }
    }
    ylabel: "m"
  }
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.angular_error"
      }
    }
    ylabel: "rad / sec"
  }
}

figure {
  axes {
    line {
      y_signal {
        channel: "DrivetrainOutput"
        field: "left_voltage"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainOutput"
        field: "right_voltage"
      }
    }
    ylabel: "V"
  }
  axes {
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.left_voltage_error"
      }
    }
    line {
      y_signal {
        channel: "DrivetrainStatus"
        field: "localizer.right_voltage_error"
      }
    }
    ylabel: "V"
  }
}
