package bot3.control_loops;

import "aos/common/controls/control_loops.q";

queue_group IntakeQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Positive = suck, negative = spit, zero = stationary.
    int16_t movement;
  };

  message Position {};

  message Output {
    double intake;
  };

  message Status {};

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group IntakeQueue intake_queue;
