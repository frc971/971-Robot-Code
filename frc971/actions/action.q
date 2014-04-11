package frc971.actions;

interface StatusInterface {
  // 0 if the action isn't running or the value from goal.run.
  uint32_t running;
};

interface GoalInterface {
  // 0 to stop or an arbitrary value to put in status.running.
  uint32_t run;
};

message Status {
  uint32_t running;
};

message Goal {
  uint32_t run;
};

interface ActionQueueGroup {
  queue Status status;
  queue Goal goal;
};
