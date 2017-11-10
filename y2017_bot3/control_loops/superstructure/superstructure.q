package y2017_bot3.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

queue_group SuperstructureQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // Voltage from -12 to 12 to send to the rollers. Positive is front surface
    // of the roller is moving down when gear is moving inwards.
    float voltage_rollers;
    // State of finger pistons. True is out, false is in.
    bool fingers_out;
    // Voltage from -12 to 12 sent to the hanger roller. Positive is front
    // surface of the hanger is moving down.
    float hanger_voltage;
  };

  message Status {
  };

  message Position {
  };

  message Output {
    // see above
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
