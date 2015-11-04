package y2014_bot3.control_loops;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

queue_group RollersQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // -1 = back intake, 1 = front intake, all else = stationary.
    int16_t intake;
    // -1 = backwards, 1 = forwards, all else = stationary.
    int16_t low_spit;
    // Whether we want the human player load function.
    bool human_player;
  };

  message Position {};

  message Output {
    // Positive voltage = intaking, Negative = spitting.
    double front_intake_voltage;
    double back_intake_voltage;
    // Voltage for the low goal rollers.
    // Positive voltage = ball towards back, Negative = ball towards front.
    double low_goal_voltage;

    // Whether the front and back intake pistons are extended.
    bool front_extended;
    bool back_extended;
  };

  message Status {};

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group RollersQueue rollers_queue;
