package y2016.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    double value;
  };

  message Status {
    double value;
  };

  message Position {
    double value;
  };

  queue Goal goal;
  queue Position position;
  queue aos.control_loops.Output output;
  queue Status status;
};

queue_group SuperstructureQueue superstructure_queue;
