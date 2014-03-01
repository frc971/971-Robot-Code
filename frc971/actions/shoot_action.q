package frc971.actions;

queue_group ShootActionQueueGroup {
  message Status {
    bool running;
  };

  message Goal {
    // If true, run this action.  If false, cancel the action if it is
    // currently running.
    bool run; // Shot power in joules.
    double shot_power;
    // Claw angle when shooting.
    bool shot_angle;
  };

  queue Goal goal;
  queue Status status;
};

queue_group ShootActionQueueGroup shoot_action;
