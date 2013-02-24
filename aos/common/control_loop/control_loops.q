package aos.control_loops;

interface IsDone {
  bool done;
};

interface ControlLoop {
  queue goal;
  queue position;
  queue output;
  queue IsDone status;
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
