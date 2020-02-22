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
        field: "hood.position"
      }
    }
    line {
      y_signal {
        channel: "Goal"
        field: "hood.unsafe_goal"
      }
    }
    line {
      y_signal {
        channel: "Position"
        field: "hood.encoder"
      }
    }
    ylabel: "hood position"
  }
}
