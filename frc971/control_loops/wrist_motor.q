package frc971.control_loops;

import "aos/common/control_loop/control_loops.q";

queue_group WristLoop {
  implements aos.control_loops.ControlLoop;

  message Position {
    double pos;
    bool hall_effect;
    // The exact pos when hall_effect last changed
    double calibration;
  };

  queue aos.control_loops.Goal goal;
  queue Position position;
  queue aos.control_loops.Output output;
  queue aos.control_loops.Status status;
};

queue_group WristLoop wrist;
