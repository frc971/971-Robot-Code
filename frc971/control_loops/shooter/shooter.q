package frc971.control_loops;

import "aos/common/control_loop/control_looops.q";

queue_group ShooterLoop {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // The energy to load to in joules.
    double energy;
    // Shoots as soon as this is true.
    bool shoot;
  };
  message Position {
    bool back_hall_effect;
    // In meters, out is positive.
    double position;
    double back_calibration;
  };
  message Status {
    // Whether it's ready to shoot right now.
    bool ready;
    // Whether the plunger is in and out of the way of grabbing a ball.
    bool cocked;
    // How many times we've shot.
    int shots;
  };

  queue Goal goal;
  queue Position position;
  queue aos.control_loops.Output output;
  queue Status status;
};

queue_group ShooterLoop shooter;
