package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

queue_group ShooterLoop {
  implements aos.control_loops.ControlLoop;

  message Goal {
    double velocity;
  };

  message Status {
    bool ready;
    double average_velocity;
  };

  message Position {
    double position;
  };

  queue Goal goal;
  queue Position position;
  queue aos.control_loops.Output output;
  queue Status status;
};

queue_group ShooterLoop shooter;
