package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

queue_group ShooterLoop {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Goal velocity in rad/sec
    double velocity;
  };

  message Status {
    // True if the shooter is up to speed.
    bool ready;
    // The average velocity over the last 0.1 seconds.
    double average_velocity;
  };

  message Position {
    // The angle of the shooter wheel measured in rad/sec.
    double position;
  };

  queue Goal goal;
  queue Position position;
  queue aos.control_loops.Output output;
  queue Status status;
};

queue_group ShooterLoop shooter;
