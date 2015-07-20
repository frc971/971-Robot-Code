package aos.common.actions;

interface StatusInterface {
  // 0 if the action isn't running or the value from goal.run.
  uint32_t running;
};

interface GoalInterface {
  // 0 to stop or an arbitrary value to put in status.running.
  uint32_t run;
};

message Status {
  // The run value of the instance we're currently running or 0.
  uint32_t running;
  // A run value we were previously running or 0.
  uint32_t last_running;
  // If false the action failed to complete and may be in a bad state,
  // this is a critical problem not a cancellation.
  bool success;
};

message Goal {
  // The unique value to put into status.running while running this instance or
  // 0 to cancel.
  uint32_t run;
  // Default parameter.  The more useful thing to do would be to define your own
  // goal type to change param to a useful structure.
  double params;
};

interface ActionQueueGroup {
  queue Status status;
  queue Goal goal;
};
