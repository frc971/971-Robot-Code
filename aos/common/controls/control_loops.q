package aos.control_loops;

interface ControlLoop {
  queue goal;
  queue position;
  queue output;
  queue status;
};

message Goal {
  double goal;
};

message Position {
  double position;
};

message Output {
  double voltage;
};

message Status {
  bool done;
};

// Single Input Single Output control loop.
queue_group SISO {
  implements ControlLoop;

  queue Goal goal;
  queue Position position;
  queue Output output;
  queue Status status;
};
