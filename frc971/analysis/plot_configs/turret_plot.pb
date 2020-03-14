channel {
  name: "/aos/roborio"
  type: "aos.JoystickState"
  alias: "JoystickState"
}
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
        field: "turret.goal_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "turret.unprofiled_goal_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "aimer.turret_velocity"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "turret.velocity"
      }
    }
  }
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "turret.goal_position"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "turret.unprofiled_goal_position"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "aimer.turret_position"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "turret.position"
      }
    }
  }
}

figure {
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "aimer.aiming_for_inner_port"
      }
    }
#    line {
#      y_signal {
#        channel: "JoystickState"
#        field: "alliance"
#      }
#    }
  }
  axes {
    line {
      y_signal {
        channel: "Status"
        field: "aimer.shot_distance"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "aimer.target_distance"
      }
    }
  }
}

figure {
  axes {
    line {
      y_signal {
        channel: "Output"
        field: "turret_voltage"
      }
    }
    line {
      y_signal {
        channel: "Status"
        field: "turret.voltage_error"
      }
    }
  }
}
