package bot3.control_loops;

import "aos/common/control_loop/control_loops.q";

queue_group ShooterLoop {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Goal velocity in rad/sec
    double velocity;
    // PWM output to intake.
    double intake;
    // Whether to activate pusher piston.
    bool push;
  };

  message Status {
    // True if the shooter is up to speed.
    bool ready;
    // The average velocity over the last 0.1 seconds.
    double average_velocity;
    // True if Frisbees are being fired into the shooter.
    bool firing;
  };

  message Position {
    // The speed of the shooter wheel measured in rad/sec.
    double velocity;
  };

  message Output {
    // Output to shooter, Volts.
    double voltage;
    // PWM output to intake.
    double intake;
    // Whether to activate pusher piston.
    bool push;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group ShooterLoop shooter;
