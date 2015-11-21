package y2015_bot3.control_loops;

import "aos/common/controls/control_loops.q";

queue_group IntakeQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Units: volts
    double movement;

    bool claw_closed;
  };

  message Position {};

  message Output {
    // Positive or negative, depending on whether we're sucking or spitting.
    double intake;

    bool claw_closed;
  };

  message Status {};

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group IntakeQueue intake_queue;
