channel {
  name: "/superstructure"
  type: "y2020.control_loops.superstructure.Status"
  alias: "Status"
}
channel {
  name: "/superstructure"
  type: "y2020.control_loops.superstructure.Output"
  alias: "Output"
}
channel {
  name: "/superstructure"
  type: "y2020.control_loops.superstructure.Position"
  alias: "Position"
}
channel {
  name: "/superstructure"
  type: "y2020.control_loops.superstructure.Goal"
  alias: "Goal"
}

figure {
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "shooter.accelerator_left.avg_angular_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "shooter.accelerator_right.avg_angular_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "shooter.accelerator_left.angular_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "shooter.accelerator_right.angular_velocity"
      }
    }
    line {
      y_signal {
        channel: "Goal"
        field: "shooter.velocity_accelerator"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "shooter.finisher.avg_angular_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "shooter.finisher.angular_velocity"
      }
    }
    line {
      y_signal {
        channel: "Goal"
        field: "shooter.velocity_finisher"
      }
    }
  }
  axes {
    line {
      y_signal {
        channel: "Output"
        field: "accelerator_left_voltage"
      }
    }
    line {
      y_signal {
        channel: "Output"
        field: "accelerator_right_voltage"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "shooter.accelerator_left.voltage_error"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "shooter.accelerator_right.voltage_error"
      }
    }
    line {
      y_signal {
        channel: "Output"
        field: "finisher_voltage"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "shooter.finisher.voltage_error"
      }
    }
    ylabel: "hood position"
  }
}
