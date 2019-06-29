package y2012.control_loops;

import "aos/controls/control_loops.q";

// Published on ".y2012.control_loops.accessories_queue"
queue_group AccessoriesQueue {
  implements aos.control_loops.ControlLoop;
  message Message {
    bool[3] solenoids;
    double[2] sticks;
  };

  queue Message goal;
  queue .aos.control_loops.Position position;
  queue Message output;
  queue .aos.control_loops.Status status;
};
