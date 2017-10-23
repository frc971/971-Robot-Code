package y2017_bot3.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Voltage to send to the rollers. Positive is sucking in.
    float voltage_rollers;
    bool fingers_out;
    float hanger_voltage;
 };

  message Status {
  };

  message Position {
  };

  message Output {
    float voltage_rollers;
    bool fingers_out;
    float hanger_voltage;
 };

  queue Goal goal;
  queue Output output;
  queue Status status;
  queue Position position;
};

queue_group SuperstructureQueue superstructure_queue;

