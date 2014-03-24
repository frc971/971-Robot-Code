package frc971.actions;

queue_group ShootActionQueueGroup {
  message Status {
    bool running;
  };

  message Goal {
    // If true, run this action.  If false, cancel the action if it is
    // currently running.
    bool run;
  };

  queue Goal goal;
  queue Status status;
};

queue_group ShootActionQueueGroup shoot_action;
