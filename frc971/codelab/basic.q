package frc971.codelab;

import "aos/common/controls/control_loops.q";
import "frc971/control_loops/control_loops.q";

// The theme of this basic test is a simple intake system.
//
// The system will have a motor driven by the voltage returned
// by output, and then eventually this motor, when run enough,
// will trigger the limit_sensor. The hypothetical motor should shut
// off in that hypothetical situation to avoid hypothetical burnout.
queue_group BasicQueue {
  implements aos.control_loops.ControlLoop;

  message Goal {
    // The control loop needs to intake now.
    bool intake;
  };

  message Position {
    // This is a potential incoming sensor value letting us know
    // if we need to be intaking.
    bool limit_sensor;
  };

  message Status {
    // Lets consumers of basic_queue.status know if
    // the requested intake is finished.
    bool intake_complete;
  };

  message Output {
    // This would be set up to drive a hypothetical motor that would
    // hope to intake something.
    double intake_voltage;
  };

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};

queue_group BasicQueue basic_queue;
